/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 *
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <dbus/dbus.h>

//#define PRINT_MSG

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 100000;
static uint16_t delay = 0;
int fd2;
int state;

static DBusMessage *create_property_get_message(const char *bus_name, const char *path, const char *iface, const char *propname);
static char extract_bool_from_variant(DBusMessage *reply, DBusError *error);
static char get_bool_property(DBusConnection *connection, const char *bus_name, const char *path, const char *iface, const char *propname, DBusError *error);
static void extract_string_from_variant(DBusMessage *reply, DBusError *error, char **result);
static void get_string_property(DBusConnection *connection, const char *bus_name, const char *path, const char *iface, const char *propname, DBusError *error, char **result);

uint8_t j1850_crc(uint8_t *msg_buf, int8_t nbytes);
static uint8_t xferbyte(int fd, uint8_t outbyte);

static char get_bool_property(DBusConnection *connection, const char *bus_name, const char *path, const char *iface, const char *propname, DBusError *error) {
    DBusError myError;
    char result = 0;
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
        return 0;
    }
 
    result = extract_bool_from_variant(replyMessage, &myError);
    if (dbus_error_is_set(&myError)) {
        dbus_move_error(&myError, error);
        return 0;
    }
 
    dbus_message_unref(replyMessage);
     
    return result;
}

static char extract_bool_from_variant(DBusMessage *reply, DBusError *error) {
    DBusMessageIter iter;
    DBusMessageIter sub;
    char result;
     
    dbus_message_iter_init(reply, &iter);
 
    if (DBUS_TYPE_VARIANT != dbus_message_iter_get_arg_type(&iter)) {
        dbus_set_error_const(error, "reply_should_be_variant", "This message hasn't a variant response type");
        return 0;
    }
 
    dbus_message_iter_recurse(&iter, &sub);
 
    if (DBUS_TYPE_BOOLEAN != dbus_message_iter_get_arg_type(&sub)) {
        dbus_set_error_const(error, "variant_should_be_boolean", "This variant reply message must have boolean content");
        return 0;
    }
 
    dbus_message_iter_get_basic(&sub, &result);
    return result;
}

static void get_string_property(DBusConnection *connection, const char *bus_name, const char *path, const char *iface, const char *propname, DBusError *error, char **result) {
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
 
    extract_string_from_variant(replyMessage, &myError, result);
    if (dbus_error_is_set(&myError)) {
        dbus_move_error(&myError, error);
        return;
    }
 
    dbus_message_unref(replyMessage);
}

static void extract_string_from_variant(DBusMessage *reply, DBusError *error, char **result) {
    DBusMessageIter iter;
    DBusMessageIter sub;
     
    dbus_message_iter_init(reply, &iter);
 
    if (DBUS_TYPE_VARIANT != dbus_message_iter_get_arg_type(&iter)) {
        dbus_set_error_const(error, "reply_should_be_variant", "This message hasn't a variant response type");
        return;
    }
 
    dbus_message_iter_recurse(&iter, &sub);
 
    if (DBUS_TYPE_STRING != dbus_message_iter_get_arg_type(&sub)) {
        dbus_set_error_const(error, "variant_should_be_string", "This variant reply message must have string content");
        return;
    }
 
    dbus_message_iter_get_basic(&sub, result);
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

void dbus_play(DBusConnection *connection) {
	DBusMessage* msg;
	DBusPendingCall* pending;

	msg = dbus_message_new_method_call("org.bluez", // target for the method call
	"/org/bluez/hci0/dev_64_BC_0C_F9_8C_4E/player0", // object to call on
	"org.bluez.MediaPlayer1", // interface to call on
	"Play"); // method name
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

void dbus_pause(DBusConnection *connection) {
	DBusMessage* msg;
	DBusPendingCall* pending;

	msg = dbus_message_new_method_call("org.bluez", // target for the method call
	"/org/bluez/hci0/dev_64_BC_0C_F9_8C_4E/player0", // object to call on
	"org.bluez.MediaPlayer1", // interface to call on
	"Pause"); // method name
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

static int send_info(int fd, char *text, uint8_t field) {
	int character = 0;
	int msg_char = 0;
	int msgs = 0;
	uint8_t msg[9][10] = {0};
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
		
		msg[msgs][0] = 0x06;
		msg[msgs][1] = 0x07;
		msg[msgs][2] = 0xAB;
		msg[msgs][9] = 0x00;
		
		msg_char = 0;
		msgs ++;
	}
	
	msg[0][3] += 0x08;
	
	int sendmsg;
	for(sendmsg=0; sendmsg<msgs; sendmsg++) {
		msg[sendmsg][3] += 0x10 * (msgs-sendmsg) + field;
		msg[sendmsg][8] = j1850_crc(&msg[sendmsg][2], 6);
		
		uint8_t i;
		for(i=0; i<10; i++) xferbyte(fd, msg[sendmsg][i]);
	}
	
	return 0;
}

int get_switches(int fd) {
	uint8_t msg[] = {0x01, 0x00};
	
	uint8_t i;
	uint8_t ret;
	for(i=0; i<sizeof(msg)+1; i++) ret = xferbyte(fd, msg[i]);
	
	return (int)ret;
}

void set_listen_headers(int fd, uint8_t *headers) {
	uint8_t msg[18] = {0};
	int nheaders = 0;
	
	msg[0] = 0x07;
	
	while(headers[nheaders] != 0) {
		msg[2+nheaders] = headers[nheaders];
		nheaders++;
	}
	
	msg[1] = nheaders;
	
	uint8_t i;
	for(i=0; i<(nheaders+3); i++) xferbyte(fd, msg[i]);
}

uint8_t j1850_crc(uint8_t *msg_buf, int8_t nbytes) {
	uint8_t crc_reg=0xff,poly,byte_count,bit_count;
	uint8_t *byte_point;
	uint8_t bit_point;

	for (byte_count=0, byte_point=msg_buf; byte_count<nbytes; ++byte_count, ++byte_point)
	{
		for (bit_count=0, bit_point=0x80 ; bit_count<8; ++bit_count, bit_point>>=1)
		{
			if (bit_point & *byte_point)	// case for new bit = 1
			{
				if (crc_reg & 0x80)
					poly=1;	// define the polynomial
				else
					poly=0x1c;
				crc_reg= ( (crc_reg << 1) | 1) ^ poly;
			}
			else		// case for new bit = 0
			{
				poly=0;
				if (crc_reg & 0x80)
					poly=0x1d;
				crc_reg= (crc_reg << 1) ^ poly;
			}
		}
	}
	return ~crc_reg;	// Return CRC
}

static uint8_t xferbyte(int fd, uint8_t outbyte) {
	int ret;
	uint8_t tx[] = {outbyte};
	uint8_t rx[] = {0};
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = 0,
		.bits_per_word = 0,
	};
	
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == -1)
		pabort("can't send spi message");
		
	return rx[0];
}

static int checkpwr(int fd)
{
	uint8_t pin;
	
	pin = xferbyte(fd, 0x02);
	pin = xferbyte(fd, 0x00);
	if(pin != 0x02) return -1;
	pin = xferbyte(fd, 0x00);

	return pin & 1;
}

void sig_handler(int sig) {
	if(sig == SIGINT) state = 0xFF;
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd = -1;

	while(fd < 0) {
		fd = open(device, O_RDWR);
		if (fd < 0) {
			printf("can't open spi device");
			usleep(500000);
		}
	}
	
	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1) pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1) pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) pabort("can't get max speed hz");
	
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
    
	dbus_play(connection);
	
//	uint8_t headers[] = {0x8D, 0x80, 0x3D, 0x00};
	uint8_t headers[] = {0x00};
	set_listen_headers(fd, headers);
	
	int info_tmr = 0;
	int last_sw = 0;
	
	state = 0;
	while(1) {
		uint8_t ret;
		uint8_t command;
		uint8_t bytes;
		
		info_tmr++;
		
		uint8_t bus;
		for(bus = 0; bus < 2; bus++) {
			command = bus + 3;
			xferbyte(fd, command);
			command = xferbyte(fd, 0x00);
			bytes = xferbyte(fd, 0x00);
			
			//if(bytes > 0) printf("\n");
			uint8_t text[12] = { 0 };
			for(int i=0; i<bytes; i++) {
				ret = xferbyte(fd, 0x00);

				if (bytes > 0) {
					if(i == bytes-1) text[11] = ret;
					else text[i] = ret;
					//printf("%.2X ", ret);
				}
			}
			
			if(bytes) {
#ifndef PRINT_MSG
				if(text[0] == 0x8D && text[1] == 0x0F) {
					uint8_t msg[12];
					state = 1;
					if(text[2] == 0x26) {
						msg[0] = 0x05;
						msg[1] = 0x06;
						msg[2] = 0x8D;
						msg[3] = 0x22;
						msg[4] = 0x11;
						msg[5] = 0x01;
						msg[6] = 0xFF;
						msg[7] = j1850_crc(&msg[2], 5);
						msg[8] = 0x00;
						//printf("\n");
						uint8_t i;
						for(i=0; i<9; i++) {
							xferbyte(fd, msg[i]);
							//printf("%.2X ", msg[i]);
						}
						msg[0] = 0x06;
						//printf("\n");
						for(i=0; i<9; i++) {
							xferbyte(fd, msg[i]);
							//printf("%.2X ", msg[i]);
						}
					}
					else {
						state = 0;
						msg[0] = 0x05;
						msg[1] = 0x06;
						msg[2] = 0x8D;
						msg[3] = 0x22;
						msg[4] = 0x10;
						msg[5] = 0x00;
						msg[6] = 0x01;
						msg[7] = j1850_crc(&msg[2], 5);
						msg[8] = 0x00;
						//printf("\n");
						uint8_t i;
						for(i=0; i<9; i++) {
							xferbyte(fd, msg[i]);
							//printf("%.2X ", msg[i]);
						}
					}
				}
#endif	
#ifdef PRINT_MSG
				uint8_t priority = (text[0] & 0b11100000) >> 5;
				uint8_t headertype = 3;
				if((text[0] & 0b00010000) > 0) headertype = 1;
				char ifr[] = "N";
				if((text[0] & 0b00001000) > 0) ifr[0] = 'N';
				uint8_t addr = ((text[0] & 0b00000100) >> 2) ^ 0x01;
				headertype = headertype - addr;
				char addressing[] = "F";
				if(!addr) addressing[0] = 'P';
				uint8_t msg_type = text[0] & 0b00000011;
				
				char target[] = "--";
				char source[] = "--";
				if(headertype == 3) {
					snprintf(target, sizeof(target), "%.2X", text[1]);
					snprintf(source, sizeof(source), "%.2X", text[2]);
				}
				else if(headertype == 2) {
					snprintf(target, sizeof(target), "%.2X", text[1]);
				}

				char output[256] = {0};
				
				snprintf(output, sizeof(output), "BUS: %1u - HDR: %.2X (P%2u HL%1u IFR: %s ADR: %s TP%1u T%s S%s) ", bus, text[0], priority, headertype, ifr, addressing, msg_type, target, source);
				
				char message[64] = {0};
				strcat(message, "MSG: ");
				uint8_t mbytes = bytes - headertype - 1;
				uint8_t i;
				for(i = 0; i < mbytes; i++) {
					char tempstr[8] = {0};
					
					snprintf(tempstr, sizeof(tempstr), "%.2X ", text[i+headertype]);
					strcat(message, tempstr);
				}
				for(i=i; i<(12); i++) {
					strcat(message, "   ");
				}
				strcat(message, "[");
				for(i = 0; i < mbytes; i++) {
					char tempstr[8] = {0};
					
					snprintf(tempstr, sizeof(tempstr), "%c", text[i+headertype]);
					strcat(message, tempstr);
				}
				for(i=i; i<(12); i++) {
					strcat(message, " ");
				}
				strcat(message, "] ");
				strcat(output, message);
				
				char crc[8] = {0};
				snprintf(crc, sizeof(crc), "CRC: %.2X", text[11]);
				
				strcat(output, crc);
			
				printf("\n%s", output);
#endif
			}

			fflush(stdout);
		}
		
#ifndef PRINT_MSG
		if(state) {
			if(info_tmr > 20) {
				info_tmr = 0;
				/*
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
				*/
				send_info(fd, "Playing Bluetooth", 0x00);
				send_info(fd, "", 0x02);
			}
		}
		
		int sw = get_switches(fd);
		/*
		if(sw != last_sw) {
			last_sw = sw;
			uint8_t msg[8];
			msg[0] = 0x05;
			msg[1] = 0x05;
			msg[2] = 0x3D;
			msg[3] = 0x11;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[7] = 0x00;
			switch(sw & 0x0F) {
				case 0x01:
					msg[4] = 0x00;
					msg[5] = 0x02;
					break;
				case 0x02:
					msg[4] = 0x20;
					msg[5] = 0x00;
					break;
				case 0x03:
					msg[4] = 0x40;
					msg[5] = 0x00;
					break;
				case 0x04:
					msg[4] = 0x10;
					msg[5] = 0x00;
					break;
				case 0x05:
					msg[4] = 0x20;
					msg[5] = 0x00;
					break;
			}
			msg[6] = j1850_crc(&msg[2], 5);
			msg[7] = 0x00;
			uint8_t i;
			for(i=0; i<8; i++) {
				xferbyte(fd, msg[i]);
				printf("%.2X ", msg[i]);
			}
			printf("\n");
		}
		*/
		
		ret = checkpwr(fd);
		if(ret < 0) printf("Error checking power pin status\n");
		else if(ret) {
			if( access( "/home/pi/pwroff", F_OK ) != -1 ) {
				remove("/home/pi/pwroff");
			}
		}
		else {
			if( access( "/home/pi/pwroff", F_OK ) == -1 ) {
				fd2 = open("/home/pi/pwroff", O_RDWR | O_CREAT, S_IRUSR | S_IRGRP | S_IROTH);
			}
		}
#endif

		if(state == 0xFF) {
			printf("\n\rQuitting...\n\r");
			break;
		}
		
		usleep(10000);
	}

	close(fd);
	
	dbus_pause(connection);
	
	dbus_connection_unref(connection);

	if( access( "/home/pi/pwroff", F_OK ) != -1 ) {
		remove("/home/pi/pwroff");
	}

	return ret;
}
