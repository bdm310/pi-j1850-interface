/*
 * Daemon for SPI connected AVR
 * Manages J1850 busses, notifying systemd that it's time to shutdown,
 * getting bluetooth information, handling switch events, ...
 */

#define _XOPEN_SOURCE 700

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <dbus/dbus.h>

#define PWR_FILE_PATH "/home/pi/pwroff"

static const char *device = "/dev/spidev0.0";
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 100000;
static uint16_t delay = 0;

static int dbg_level;
static int listen;

static int state;

static int xferbyte(int fd, int outbyte);
static int spi_send_data(int fd, int *tx_buf, int len);
static int spi_get_data(int fd, int *rx_buf);
static int spi_get_response(int fd, int *rx_buf);
static int update_pwr_file(int pwr);
static int update_sw(int fd, int sw_state, int last_sw_state);

static DBusMessage *create_property_get_message(const char *bus_name, const char *path, const char *iface, const char *propname);

static int xferbyte(int fd, int outbyte) {
    int ret;
    int rx = 0;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)&outbyte,
        .rx_buf = (unsigned long)&rx,
        .len = 1,
        .delay_usecs = delay,
        .speed_hz = 0,
        .bits_per_word = 0,
    };
    
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    
    if(ret < 0) return ret;
    else return rx;
}

static int update_pwr_file(int pwr) {
    int fd;
    
    if(pwr) {
        if(access(PWR_FILE_PATH, F_OK) != -1 ) {
            if(remove(PWR_FILE_PATH) < 0) return -1;
        }
    }
    else {
        if(access(PWR_FILE_PATH, F_OK) == -1) {
            fd = open(PWR_FILE_PATH, O_RDWR | O_CREAT, S_IRUSR | S_IRGRP | S_IROTH);
            
            if(fd < 0) {
                printf("Error creating pwr file\n");
                return -1;
            }
            else close(fd);
        }
    }
    
    return 0;
}

static int update_sw(int fd, int sw_state, int last_sw_state) {
    int ret;
    int send_buf[] = {0x07, 0x04, 0x3D, 0x11, 0x00, 0x00};
    
    if(sw_state) {
        switch(sw_state) {
            //Off/Vol-
            case 0x01:
                send_buf[4] = 0x02;
                send_buf[5] = 0x00;
                break;
            //Cancel/Mode
            case 0x02:
                send_buf[4] = 0x00;
                send_buf[5] = 0x02;
                break;
            //Set/Seek-
            case 0x03:
                send_buf[4] = 0x10;
                send_buf[5] = 0x00;
                break;
            //Resume/Seek+
            case 0x04:
                send_buf[4] = 0x20;
                send_buf[5] = 0x00;
                break;
            //On/Vol+
            case 0x05:
                send_buf[4] = 0x04;
                send_buf[5] = 0x00;
                break;
        }
        ret = spi_send_data(fd, send_buf, 6);
    }
    else {
        ret = spi_send_data(fd, send_buf, 6);
    }
    
    return ret;
}

static int set_listen_headers(int fd, int *headers) {
    int ret;
    int data;
    int nheaders = 0;
    
    data = 0x05;
    ret = spi_send_data(fd, &data, 1);
    if(ret < 0) return ret;
    
	while(headers[nheaders] != 0) {
        data = 0x06;
        ret = spi_send_data(fd, &data, 1);
        if(ret < 0) return ret;
        ret = spi_send_data(fd, &headers[nheaders], 1);
        if(ret < 0) return ret;
		nheaders++;
	}
    
    return 0;
}

static void print_j1850_msg(int *msg, int bytes, int bus) {
    int priority = (msg[0] & 0b11100000) >> 5;
    int headertype = 3;
    if((msg[0] & 0b00010000) > 0) headertype = 1;
    char ifr[] = "N";
    if((msg[0] & 0b00001000) > 0) ifr[0] = 'N';
    int addr = ((msg[0] & 0b00000100) >> 2) ^ 0x01;
    headertype = headertype - addr;
    char addressing[] = "F";
    if(!addr) addressing[0] = 'P';
    int msg_type = msg[0] & 0b00000011;
    
    char target[] = "--";
    char source[] = "--";
    if(headertype == 3) {
        snprintf(target, sizeof(target), "%.2X", msg[1]);
        snprintf(source, sizeof(source), "%.2X", msg[2]);
    }
    else if(headertype == 2) {
        snprintf(target, sizeof(target), "%.2X", msg[1]);
    }

    char output[256] = {0};
    
    snprintf(output, sizeof(output), "BUS: %1i - HDR: %.2X (P%2i HL%1i IFR: %s ADR: %s TP%1i T%s S%s) ", bus, msg[0], priority, headertype, ifr, addressing, msg_type, target, source);
    
    char message[64] = {0};
    strcat(message, "MSG: ");
    int mbytes = bytes - headertype - 1;
    int i;
    for(i = 0; i < mbytes; i++) {
        char tempstr[8] = {0};
        
        snprintf(tempstr, sizeof(tempstr), "%.2X ", msg[i+headertype]);
        strcat(message, tempstr);
    }
    for(i=i; i<(12); i++) {
        strcat(message, "   ");
    }
    strcat(message, "[");
    for(i = 0; i < mbytes; i++) {
        char tempstr[8] = {0};
        
        snprintf(tempstr, sizeof(tempstr), "%c", msg[i+headertype]);
        strcat(message, tempstr);
    }
    for(i=i; i<(12); i++) {
        strcat(message, " ");
    }
    strcat(message, "] ");
    strcat(output, message);
    
    char crc[8] = {0};
    snprintf(crc, sizeof(crc), "CRC: %.2X", msg[11]);
    
    strcat(output, crc);
			
    printf("%s\n", output);
}

static int get_j1850_msg(int fd, int bus, int *data) {
    int ret;
    int i;
    int rx_buf[64];
    int tx_buf;
    
    //Clear send buffer on micro
    do {
        ret = spi_get_data(fd, rx_buf);
    } while(ret > 0);
    if(ret < 0) return ret;
    
    //Request the next message in the buffer
    if(bus == 0) tx_buf = 0x03;
    else tx_buf = 0x04;
    ret = spi_send_data(fd, &tx_buf, 1);
    if(ret < 0) return ret;
    
    //Wait for and get the micro's response
    ret = spi_get_response(fd, rx_buf);
    if(ret < 0) return ret;
    ret --;
    
    if(rx_buf[0] != 0x00) {
        //Copy to output, discarding status byte
        for(i=0; i<ret; i++) data[i] = rx_buf[i+1];
        
        if(dbg_level) print_j1850_msg(data, ret, bus);
    }
    
    return ret;
}

static int spi_send_data(int fd, int *tx_buf, int len) {
    int ret;
    
    int i;
    for(i=0; i<len; i++) {
        //Send write command
        ret = xferbyte(fd, 0x02);
        if(ret < 0) return ret;
        
        //Send data byte and verify command/data
        ret = xferbyte(fd, tx_buf[i]);
        if(ret != 0x02) {
            printf("Command didn't match request, wanted %i, got %i\n", 0x02, ret);
            return -1;
        }
        ret = xferbyte(fd, 0x00);
        if(ret != tx_buf[i]) {
            printf("Data didn't match request, wanted %i, got %i\n", tx_buf[i], ret);
            return -1;
        }
    }
    
    return 0;
}

static int spi_get_data(int fd, int *rx_buf) {
    int ret;
    int len = 0;
    
    //Clear SPDR and get buffer status
    ret = xferbyte(fd, 0x00);
    if(ret < 0) return ret;
    ret = xferbyte(fd, 0x00);
    if(ret < 0) return ret;
    
    while(ret) {
        //Send and verify read command
        ret = xferbyte(fd, 0x01);
        if(ret < 0) return ret;
        ret = xferbyte(fd, 0x00);
        if(ret != 1) return -1;
        
        //Get actual data byte
        ret = xferbyte(fd, 0x00);
        if(ret < 0) return ret;
        rx_buf[len] = ret;
        len ++;
        
        //Get buffer status
        ret = xferbyte(fd, 0x00);
        if(ret < 0) return ret;
    }
    
    return len;
}

static int spi_get_response(int fd, int *rx_buf) {
    int ret;
    struct timespec start, end;
    
    //Wait for the micro to respond to our request
    clock_gettime(CLOCK_MONOTONIC, &start);
    do {
        ret = spi_get_data(fd, rx_buf);
        if(ret < 1) return ret;
        
        //Give up after 100ms
        clock_gettime(CLOCK_MONOTONIC, &end);
        if(end.tv_nsec - start.tv_nsec > 100000000) return -1;
    } while(ret == 0);
    
    return ret;
}

void sig_handler(int sig) {
    if(sig == SIGINT) state = 0xFF;
}

static DBusMessage *create_property_get_message(const char *bus_name, const char *path, const char *iface, const char *propname) {
    DBusMessage *queryMessage = NULL;
 
    queryMessage = dbus_message_new_method_call(bus_name, path, 
                            "org.freedesktop.DBus.Properties",
                            "Get");
    dbus_message_append_args(queryMessage,
                 DBUS_TYPE_STRING, &iface,
                 DBUS_TYPE_STRING, &propname,
                 DBUS_TYPE_INVALID);
 
    return queryMessage;
}

static void get_track_parameter(DBusConnection *connection, const char *bus_name, const char *path, const char *iface, const char *propname, DBusError *error, char *search_key, char **result) {
    DBusError myError;
    DBusMessage *queryMessage = NULL;
    DBusMessage *replyMessage = NULL;
 
    dbus_error_init(&myError);
     
    queryMessage = create_property_get_message(bus_name, path, iface, propname);
    replyMessage = dbus_connection_send_with_reply_and_block(connection,
                          queryMessage,
                          1000,
                          &myError);
    dbus_message_unref(queryMessage);
    if (dbus_error_is_set(&myError)) {
        dbus_move_error(&myError, error);
        return;
    }
     
    DBusMessageIter iter;
    DBusMessageIter sub;
    DBusMessageIter entry;
     
    dbus_message_iter_init(replyMessage, &iter);
 
    if (DBUS_TYPE_VARIANT != dbus_message_iter_get_arg_type(&iter)) {
        dbus_set_error_const(error, "reply_should_be_variant", "This message hasn't a variant entry response type");
        return;
    }
 
	//Move in to array of response (array)
    dbus_message_iter_recurse(&iter, &iter);
	
	//Move in to dictionary (dictionary)
	dbus_message_iter_recurse(&iter, &sub);
	
	do {
		//Get entry, initialized on key (string)
		dbus_message_iter_recurse(&sub, &entry);
		
		char *key = NULL;
		dbus_message_iter_get_basic(&entry, &key);
		
		int character = 0;
		int match = 1;
		while(key[character] != '\0' && search_key[character] != '\0') {
			if(key[character] != search_key[character]) {
				match = 0;
				break;
			}
			character ++;
		}
		if(match) {
			DBusMessageIter value;
			
			//Increment entry to value (variant)
			dbus_message_iter_next(&entry);
			dbus_message_iter_recurse(&entry, &value);
			dbus_message_iter_get_basic(&value, result);
		}
	} while (dbus_message_iter_next(&sub));
 
    dbus_message_unref(replyMessage);
}

static int send_info(int fd, char *text, uint8_t field) {
	int character = 0;
	int msg_char = 0;
	int msgs = 0;
	int msg[9][8] = {0};
	int maxchar;
	
	switch(field) {
		case 0x00:
			maxchar = 20;
			break;
		case 0x01:
			maxchar = 18;
			break;
		case 0x02:
			maxchar = 8;
			break;
		case 0x04:
			maxchar = 36;
			break;
		case 0x05:
			maxchar = 36;
			break;
		default:
			return -1;
	}
	
	while(text[character] != '\0' && character < maxchar) {
		while(text[character] != '\0' && character < maxchar && msg_char < 4) {
			msg[msgs][msg_char+4] = text[character];
			msg_char ++;
			character ++;
		}
		for(; msg_char<4; msg_char++) msg[msgs][msg_char+4] = 0x20;
		
		msg[msgs][0] = 0x08;
		msg[msgs][1] = 0x06;
		msg[msgs][2] = 0xAB;
		
		msg_char = 0;
		msgs ++;
	}
	
	msg[0][3] += 0x08;
	
	int sendmsg;
	for(sendmsg=0; sendmsg<msgs; sendmsg++) {
		msg[sendmsg][3] += 0x10 * (msgs-sendmsg) + field;
		
		spi_send_data(fd, &msg[sendmsg][0], 8);
	}
	
	return 0;
}

void dbus_method(DBusConnection *connection, char *method) {
	DBusMessage* msg;
	DBusPendingCall* pending;

	msg = dbus_message_new_method_call("org.bluez", // target for the method call
	"/org/bluez/hci0/dev_64_BC_0C_F9_8C_4E/player0", // object to call on
	"org.bluez.MediaPlayer1", // interface to call on
	method); // method name
	if (NULL == msg) { 
		fprintf(stderr, "Message Null\n");
		return;
	}

	// send message and get a handle for a reply
	if (!dbus_connection_send_with_reply (connection, msg, &pending, -1)) { // -1 is default timeout
		fprintf(stderr, "Out Of Memory!\n"); 
		return;
	}
	if (NULL == pending) { 
		fprintf(stderr, "Pending Call Null\n"); 
		return; 
	}
	dbus_connection_flush(connection);

	// free message
	dbus_message_unref(msg);
}

static int spi_init(int *fd) {
    int ret;
    
    while(*fd < 0) {
        *fd = open(device, O_RDWR);
        if (*fd < 0) {
            printf("can't open spi device\n");
            nanosleep((const struct timespec[]){{0, 500000000L}}, NULL);
        }
    }
    
    // spi mode
    ret = ioctl(*fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1) {
        printf("can't set spi mode\n");
        return ret;
    }
    ret = ioctl(*fd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1) {
        printf("can't get spi mode\n");
        return ret;
    }

    // bits per word
    ret = ioctl(*fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1) {
        printf("can't set bits per word\n");
        return ret;
    }
    ret = ioctl(*fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1) {
        printf("can't get bits per word\n");
        return ret;
    }

    // max speed hz
    ret = ioctl(*fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1) {
        printf("can't set max speed hz\n");
        return ret;
    }
    ret = ioctl(*fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1) {
        printf("can't get max speed hz\n");
        return ret;
    }
    
    return 0;
}

int main(int argc, char *argv[])
{
    int ret = 0;
    int fd = -1;
    
    dbg_level = 0;
    listen = 0;
    int opt;
    while ((opt = getopt(argc, argv, "dl")) != -1) {
        switch (opt) {
        case 'd': dbg_level = 1; break;
        case 'l': listen = 1; break;
        default:
            fprintf(stderr, "Usage: %s [-dl]\n", argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    if(spi_init(&fd) < 0) return -1;
    
    signal(SIGINT, sig_handler);
    
    DBusConnection *connection = NULL;
    DBusError error;
 
    dbus_error_init(&error);
    connection = dbus_bus_get(DBUS_BUS_SYSTEM, &error);
    if (dbus_error_is_set(&error)) {
        fprintf(stderr, "%s", error.message);
        abort();
    }
 
    puts("This is my unique name");
    puts(dbus_bus_get_unique_name(connection));
    
    int headers[8] = {0x00};
	if(!listen) {
        headers[0] = 0x8D;
        headers[1] = 0x3D;
    }
    ret = set_listen_headers(fd, headers);
    if(ret < 0) exit(EXIT_FAILURE);
    
    int tmr_10ms = 0;
    state = 0;
    int last_sw_state = 0;
    int last_state = 0;
    while(1) {
        int rx_buf[64];
        int tx_buf[64];
        int sw_state;
        
        if(state == 0xFF) break;
      
        //Get switch state
        while(spi_get_data(fd, rx_buf) > 0);
        tx_buf[0] = 0x01;
        ret = spi_send_data(fd, tx_buf, 1);
        if(ret < 0) printf("Error sending switch command: %i\n", ret);
        ret = spi_get_response(fd, rx_buf);
        if(ret < 0) printf("Error getting switch response: %i\n", ret);
        sw_state = rx_buf[0];
        
        //Do switches
        if(sw_state != last_sw_state) {
            last_sw_state = sw_state;
            
            if(dbg_level) printf("Switch byte: %.2X\n", sw_state);
            ret = update_sw(fd, sw_state, last_sw_state);
            if(ret < 0) printf("Error handling switch state: %i\n", ret);
        }
        
        //Get J1850 bus 0 messages
        ret = get_j1850_msg(fd, 0, rx_buf);
        if(ret < 0) printf("Error retrieving bus 0 messages: %i\n", ret);
        if(ret && !listen) {
            if(rx_buf[0] == 0x8D && rx_buf[1] == 0x0F) {
                if(rx_buf[2] == 0x26) {
                    if(dbg_level) printf("Sending sat active\n");
                    int send_buf[] = {0x07, 0x05, 0x8D, 0x22, 0x11, 0x01, 0x01};
                    spi_send_data(fd, send_buf, 7);
                    state = 0x01;
                }
                else {
                    if(dbg_level) printf("Sending sat exists\n");
                    int send_buf[] = {0x07, 0x05, 0x8D, 0x22, 0x10, 0x00, 0x01};
                    spi_send_data(fd, send_buf, 7);
                    state = 0x00;
                }
            }
            if(state) {
                if(rx_buf[0] == 0x3D && rx_buf[1] == 0x12 && rx_buf[2] == 0x83) {
                    if(rx_buf[3] == 0x26) dbus_method(connection, "Next");
                    else if(rx_buf[3] == 0x27) dbus_method(connection, "Previous");
                }
            }
        }
        
        //Get J1850 bus 1 messages
        ret = get_j1850_msg(fd, 1, rx_buf);
        if(ret < 0) printf("Error retrieving bus 1 messages: %i\n", ret);
        
        //250ms timer
        if(tmr_10ms > 25) {
            tmr_10ms = 0;
            
            if(sw_state == 0) update_sw(fd, 0x00, 0x00);
            
            if(state != last_state) {
                last_state = state;
                if(state) {
                    dbus_method(connection, "Play");
                }
                else {
                    dbus_method(connection, "Pause");
                }
            }
            if(state) {
                char *song = NULL;
				dbus_error_init(&error);
				get_track_parameter(connection, "org.bluez",
								 "/org/bluez/hci0/dev_64_BC_0C_F9_8C_4E/player0",
								 "org.bluez.MediaPlayer1",
								 "Track",
								 &error, "Title", &song);
				//if(dbus_error_is_set(&error)) printf("%s", error.message);
				
				if(song) send_info(fd, song, 0x04);
				
				char *album = NULL;
				dbus_error_init(&error);
				get_track_parameter(connection, "org.bluez",
								 "/org/bluez/hci0/dev_64_BC_0C_F9_8C_4E/player0",
								 "org.bluez.MediaPlayer1",
								 "Track",
								 &error, "Album", &album);
				//if(dbus_error_is_set(&error)) printf("%s", error.message);
				
				if(album) send_info(fd, album, 0x01);
				
				char *artist = NULL;
				dbus_error_init(&error);
				get_track_parameter(connection, "org.bluez",
								 "/org/bluez/hci0/dev_64_BC_0C_F9_8C_4E/player0",
								 "org.bluez.MediaPlayer1",
								 "Track",
								 &error, "Artist", &artist);
				//if(dbus_error_is_set(&error)) printf("%s", error.message);
				
				if(artist) send_info(fd, artist, 0x05);
				
				send_info(fd, "Playing Bluetooth", 0x00);
                send_info(fd, "", 0x02);
            }
        }
        
        //Do power pins
        while(spi_get_data(fd, rx_buf) > 0);
        tx_buf[0] = 0x02;
        ret = spi_send_data(fd, tx_buf, 1);
        if(ret < 0) printf("Error sending power command: %i\n", ret);
        ret = spi_get_response(fd, rx_buf);
        if(ret < 0) printf("Error getting power response: %i\n", ret);
        ret = update_pwr_file(rx_buf[0] & 0x01);
        if(ret < 0) printf("Error processing power: %i\n", ret);
        
        //Done
        fflush(stdout);
        nanosleep((const struct timespec[]){{0, 10000000L}}, NULL);
        tmr_10ms ++;
    }
    
    ret = update_pwr_file(0x01);
    if(ret < 0) printf("Error cleaning up power: %i\n", ret);
        
    close(fd);
    
    exit(EXIT_SUCCESS);
}
