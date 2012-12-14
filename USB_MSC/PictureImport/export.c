/*
 * xusb: Generic USB test program
 * Copyright (c) 2009-2011 Pete Batard <pbatard@gmail.com>
 * Based on lsusb, copyright (c) 2007 Daniel Drake <dsd@gentoo.org>
 * With contributions to Mass Storage test by Alan Stern.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "dosfs.h"

#include <libusb-1.0/libusb.h>

#if defined(_WIN32)
#define msleep(msecs) Sleep(msecs)
#else
#include <unistd.h>
#define msleep(msecs) usleep(1000*msecs)
#endif

#if !defined(_MSC_VER) || _MSC_VER<=1200
#define sscanf_s sscanf
#endif

#if !defined(bool)
#define bool int
#endif
#if !defined(true)
#define true (1 == 1)
#endif
#if !defined(false)
#define false (!true)
#endif

// Future versions of libusb will use usb_interface instead of interface
// in libusb_config_descriptor => catter for that
#define usb_interface interface

// Global variables
bool binary_dump = false;
char binary_name[64];

char *base_path = "DCIM/100APPLE";

inline static int perr(char const *format, ...)
{
        va_list args;
        int r;

        va_start(args, format);
        r = vfprintf(stderr, format, args);
        va_end(args);

        return r;
}

#define ERR_EXIT(errcode) do { perr("   %d\n", (errcode)); return -1; } while (0)
#define CALL_CHECK(fcall) do { r=fcall; if (r < 0) ERR_EXIT(r); } while (0);
#define B(x) (((x)!=0)?1:0)
#define be_to_int32(buf) (((buf)[0]<<24)|((buf)[1]<<16)|((buf)[2]<<8)|(buf)[3])

#define RETRY_MAX                     5
#define REQUEST_SENSE_LENGTH          0x12
#define INQUIRY_LENGTH                0x24
#define READ_CAPACITY_LENGTH          0x08

// HID Class-Specific Requests values. See section 7.2 of the HID specifications
#define HID_GET_REPORT                0x01
#define HID_SET_REPORT                0x09
#define HID_REPORT_TYPE_INPUT         0x01
#define HID_REPORT_TYPE_OUTPUT        0x02

// Mass Storage Requests values. See section 3 of the Bulk-Only Mass Storage Class specifications
#define BOMS_RESET                    0xFF
#define BOMS_GET_MAX_LUN              0xFE

// Section 5.1: Command Block Wrapper (CBW)
struct command_block_wrapper {
        uint8_t dCBWSignature[4];
        uint32_t dCBWTag;
        uint32_t dCBWDataTransferLength;
        uint8_t bmCBWFlags;
        uint8_t bCBWLUN;
        uint8_t bCBWCBLength;
        uint8_t CBWCB[16];
};

// Section 5.2: Command Status Wrapper (CSW)
struct command_status_wrapper {
        uint8_t dCSWSignature[4];
        uint32_t dCSWTag;
        uint32_t dCSWDataResidue;
        uint8_t bCSWStatus;
};

static uint8_t cdb_length[256] = {
        //   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
        06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, //  0
        06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, //  1
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, //  2
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, //  3
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, //  4
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, //  5
        00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, //  6
        00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, //  7
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, //  8
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, //  9
        12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, //  A
        12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, //  B
        00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, //  C
        00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, //  D
        00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, //  E
        00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, //  F
};

enum test_type {
        USE_GENERIC,
        USE_PS3,
        USE_XBOX,
        USE_SCSI,
} test_mode;
uint16_t VID, PID;

void display_buffer_hex(unsigned char *buffer, unsigned size)
{
        unsigned i, j, k;

        for (i = 0; i < size; i += 16) {
                printf("\n  %08x  ", i);
                for (j = 0, k = 0; k < 16; j++, k++) {
                        if (i + j < size) {
                                printf("%02x", buffer[i + j]);
                        } else {
                                printf("  ");
                        }
                        printf(" ");
                }
                printf(" ");
                for (j = 0, k = 0; k < 16; j++, k++) {
                        if (i + j < size) {
                                if ((buffer[i + j] < 32)
                                    || (buffer[i + j] > 126)) {
                                        printf(".");
                                } else {
                                        printf("%c", buffer[i + j]);
                                }
                        }
                }
        }
        printf("\n");
}

int
send_mass_storage_command(libusb_device_handle * handle, uint8_t endpoint,
                          uint8_t lun, uint8_t * cdb, uint8_t direction,
                          int data_length, uint32_t * ret_tag)
{
        static uint32_t tag = 1;
        uint8_t cdb_len;
        int i, r, size;
        struct command_block_wrapper cbw;

        if (cdb == NULL) {
                return -1;
        }

        if (endpoint & LIBUSB_ENDPOINT_IN) {
                perr("send_mass_storage_command: cannot send command on IN endpoint\n");
                return -1;
        }

        cdb_len = cdb_length[cdb[0]];
        if ((cdb_len == 0) || (cdb_len > sizeof(cbw.CBWCB))) {
                perr("send_mass_storage_command: don't know how to handle this command (%02X, length %d)\n", cdb[0], cdb_len);
                return -1;
        }

        memset(&cbw, 0, sizeof(cbw));
        cbw.dCBWSignature[0] = 'U';
        cbw.dCBWSignature[1] = 'S';
        cbw.dCBWSignature[2] = 'B';
        cbw.dCBWSignature[3] = 'C';
        *ret_tag = tag;
        cbw.dCBWTag = tag++;
        cbw.dCBWDataTransferLength = data_length;
        cbw.bmCBWFlags = direction;
        cbw.bCBWLUN = lun;
        // Subclass is 1 or 6 => cdb_len
        cbw.bCBWCBLength = cdb_len;
        memcpy(cbw.CBWCB, cdb, cdb_len);

        i = 0;
        do {
                // The transfer length must always be exactly 31 bytes.
                r = libusb_bulk_transfer(handle, endpoint,
                                         (unsigned char *)&cbw, 31, &size,
                                         1000);
                if (r == LIBUSB_ERROR_PIPE) {
                        libusb_clear_halt(handle, endpoint);
                }
                i++;
        }
        while ((r == LIBUSB_ERROR_PIPE) && (i < RETRY_MAX));
        if (r != LIBUSB_SUCCESS) {
                perr("   send_mass_storage_command: %d\n", (r));
                return -1;
        }

        return 0;
}

int
get_mass_storage_status(libusb_device_handle * handle, uint8_t endpoint,
                        uint32_t expected_tag)
{
        int i, r, size;
        struct command_status_wrapper csw;

        // The device is allowed to STALL this transfer. If it does, you have to
        // clear the stall and try again.
        i = 0;
        do {
                r = libusb_bulk_transfer(handle, endpoint,
                                         (unsigned char *)&csw, 13, &size,
                                         1000);
                if (r == LIBUSB_ERROR_PIPE) {
                        libusb_clear_halt(handle, endpoint);
                }
                i++;
        }
        while ((r == LIBUSB_ERROR_PIPE) && (i < RETRY_MAX));
        if (r != LIBUSB_SUCCESS) {
                perr("   get_mass_storage_status: %d\n", (r));
                return -1;
        }
        if (size != 13) {
                perr("   get_mass_storage_status: received %d bytes (expected 13)\n", size);
                return -1;
        }
        if (csw.dCSWTag != expected_tag) {
                perr("   get_mass_storage_status: mismatched tags (expected %08X, received %08X)\n", expected_tag, csw.dCSWTag);
                return -1;
        }
        // For this test, we ignore the dCSWSignature check for validity...
        if (csw.dCSWTag != expected_tag)
                return -1;
        if (csw.bCSWStatus) {
                // REQUEST SENSE is appropriate only if bCSWStatus is 1, meaning that the
                // command failed somehow.  Larger values (2 in particular) mean that
                // the command couldn't be understood.
                if (csw.bCSWStatus == 1)
                        return -2;  // request Get Sense
                else
                        return -1;
        }
        // In theory we also should check dCSWDataResidue.  But lots of devices
        // set it wrongly.
        return 0;
}

void
get_sense(libusb_device_handle * handle, uint8_t endpoint_in,
          uint8_t endpoint_out)
{
        uint8_t cdb[16];        // SCSI Command Descriptor Block
        uint8_t sense[18];
        uint32_t expected_tag;
        int size;

        // Request Sense
        printf("Request Sense:\n");
        memset(sense, 0, sizeof(sense));
        memset(cdb, 0, sizeof(cdb));
        cdb[0] = 0x03;          // Request Sense
        cdb[4] = REQUEST_SENSE_LENGTH;

        send_mass_storage_command(handle, endpoint_out, 0, cdb,
                                  LIBUSB_ENDPOINT_IN, REQUEST_SENSE_LENGTH,
                                  &expected_tag);
        libusb_bulk_transfer(handle, endpoint_in, (unsigned char *)&sense,
                             REQUEST_SENSE_LENGTH, &size, 1000);
        printf("   received %d bytes\n", size);

        if ((sense[0] != 0x70) && (sense[0] != 0x71)) {
                perr("   ERROR No sense data\n");
        } else {
                perr("   ERROR Sense: %02X %02X %02X\n", sense[2] & 0x0F,
                     sense[12], sense[13]);
        }
        // Strictly speaking, the get_mass_storage_status() call should come
        // before these perr() lines.  If the status is nonzero then we must
        // assume there's no data in the buffer.  For xusb it doesn't matter.
        get_mass_storage_status(handle, endpoint_in, expected_tag);
}

typedef struct {
        unsigned char command;  //0
        unsigned char res1;     //1
        unsigned int block;     //2-5
        unsigned char res2;     //6
        unsigned short numblocks; //7-8
        unsigned char res3;     //9 - the block is 10 bytes long
} __attribute__ ((packed)) cmdblock_t;

typedef struct {
        unsigned char command;  //0
        unsigned char res1;     //1
        unsigned char res2;     //2
        unsigned char res3;     //3
        unsigned char lun;      //4
        unsigned char res4;     //5
} __attribute__ ((packed)) cmdblock6_t;

typedef struct {
        libusb_device_handle *handle;
        uint8_t endpoint_in;
        uint8_t endpoint_out;
        uint32_t block_size;
        uint8_t lun;
} save;

save _save;

unsigned char *_read_block_number(libusb_device_handle * handle,
                                  uint8_t endpoint_in, uint8_t endpoint_out,
                                  uint32_t block_size, uint8_t lun,
                                  uint32_t * size1, int block_num);

#define DFS_ReadSector(unit,buffer,sector,count) DFS_HostReadSector(buffer,sector,count)
#define DFS_WriteSector(unit,buffer,sector,count) DFS_HostWriteSector(buffer,sector,count)

int DFS_HostWriteSector(uint8_t * buffer, uint32_t sector, uint32_t count)
{
        return 0;
}

int DFS_HostReadSector(uint8_t * buffer, uint32_t sector, uint32_t count)
{
        int size;
        unsigned char *p = _read_block_number(_save.handle, _save.endpoint_in,
                                              _save.endpoint_out,
                                              _save.block_size, _save.lun,
                                              &size, sector);
        memcpy(buffer, p, size);
        free(p);
        return 0;
}

// Mass Storage device to test bulk transfers (non destructive test)
unsigned char *_read_block_number(libusb_device_handle * handle,
                                  uint8_t endpoint_in, uint8_t endpoint_out,
                                  uint32_t block_size, uint8_t lun,
                                  uint32_t * size1, int block_num)
{
        int size;
        uint32_t expected_tag;
        unsigned char *data;
        cmdblock_t cb;
        memset(&cb, 0, sizeof(cb));
        cb.command = 0x28;
        cb.block = htonl(block_num);
        cb.numblocks = htons(1);

        data = (unsigned char *)calloc(1, block_size);
        if (data == NULL) {
                perr("   unable to allocate data buffer\n");
                return NULL;
        }

        send_mass_storage_command(handle, endpoint_out, lun, (char *)&cb,
                                  LIBUSB_ENDPOINT_IN, block_size,
                                  &expected_tag);
        libusb_bulk_transfer(handle, endpoint_in, data, block_size, &size,
                             5000);

        if (get_mass_storage_status(handle, endpoint_in, expected_tag) == -2) {
                get_sense(handle, endpoint_in, endpoint_out);
                return NULL;
        }

        *size1 = size;

        return data;
}

char *strrtok(char *str, const char *delim)
{
    int i, j;

    for (i = strlen(str) - 1; i > 0; i--)
    {
        // Sets the furthest set of contiguous delimiters to null characters
        if (strchr(delim, str[i]))
        {
            j = i + 1;

            while (strchr(delim, str[i]) && i >= 0)
            {
                str[i] = '\0';
                i--;
            }

            return &(str[j]);
        }
    }

    return str;
}

int
mass_storage(libusb_device_handle * handle, uint8_t endpoint_in,
             uint8_t endpoint_out)
{
        int r, size;
        uint8_t lun;
        uint32_t expected_tag;
        uint32_t i, max_lba, block_size;
        double device_size;
        uint8_t cdb[16];        // SCSI Command Descriptor Block
        uint8_t buffer[64];
        char vid[9], pid[9], rev[5];
        unsigned char *data;
        FILE *fd;

        r = libusb_control_transfer(handle,
                                    LIBUSB_ENDPOINT_IN |
                                    LIBUSB_REQUEST_TYPE_CLASS |
                                    LIBUSB_RECIPIENT_INTERFACE,
                                    BOMS_GET_MAX_LUN, 0, 0, &lun, 1, 1000);
        // Some devices send a STALL instead of the actual value.
        // In such cases we should set lun to 0.
        if (r == 0) {
                lun = 0;
        } else if (r < 0) {
                perr("   Failed: %d", ((enum libusb_error)r));
        }
        // Send Inquiry
        memset(buffer, 0, sizeof(buffer));
        memset(cdb, 0, sizeof(cdb));
        cdb[0] = 0x12;          // Inquiry
        cdb[4] = INQUIRY_LENGTH;

        send_mass_storage_command(handle, endpoint_out, lun, cdb,
                                  LIBUSB_ENDPOINT_IN, INQUIRY_LENGTH,
                                  &expected_tag);
        CALL_CHECK(libusb_bulk_transfer
                   (handle, endpoint_in, (unsigned char *)&buffer,
                    INQUIRY_LENGTH, &size, 1000));
        // The following strings are not zero terminated
        for (i = 0; i < 8; i++) {
                vid[i] = buffer[8 + i];
                pid[i] = buffer[16 + i];
                rev[i / 2] = buffer[32 + i / 2];  // instead of another loop
        }
        vid[8] = 0;
        pid[8] = 0;
        rev[4] = 0;
        if (get_mass_storage_status(handle, endpoint_in, expected_tag) == -2) {
                get_sense(handle, endpoint_in, endpoint_out);
        }
        // Read capacity
        memset(buffer, 0, sizeof(buffer));
        memset(cdb, 0, sizeof(cdb));
        cdb[0] = 0x25;          // Read Capacity

        send_mass_storage_command(handle, endpoint_out, lun, cdb,
                                  LIBUSB_ENDPOINT_IN, READ_CAPACITY_LENGTH,
                                  &expected_tag);
        CALL_CHECK(libusb_bulk_transfer
                   (handle, endpoint_in, (unsigned char *)&buffer,
                    READ_CAPACITY_LENGTH, &size, 1000));
        max_lba = be_to_int32(&buffer[0]);
        block_size = be_to_int32(&buffer[4]);
        device_size =
            ((double)(max_lba + 1)) * block_size / (1024 * 1024 * 1024);

        if (get_mass_storage_status(handle, endpoint_in, expected_tag) == -2) {
                get_sense(handle, endpoint_in, endpoint_out);
        }

        data = (unsigned char *)calloc(1, block_size);
        if (data == NULL) {
                perr("   unable to allocate data buffer\n");
                return -1;
        }
        // Send Read

        memset(cdb, 0, sizeof(cdb));

        _save.handle = handle;
        _save.endpoint_in = endpoint_in;
        _save.endpoint_out = endpoint_out;
        _save.block_size = block_size;
        _save.lun = lun;

        uint8_t sector[SECTOR_SIZE], sector2[SECTOR_SIZE];
        uint32_t pstart, psize;
        uint8_t pactive, ptype;
        VOLINFO vi;
        DIRINFO di;
        DIRENT de;
        uint32_t cache;
        FILEINFO fi;
        uint8_t *p;

        // Obtain pointer to first partition on first (only) unit
        pstart = DFS_GetPtnStart(0, sector, 0, &pactive, &ptype, &psize);
        if (pstart == 0xffffffff) {
                printf("Cannot find first partition\n");
                return -1;
        }
#if 0
        printf
            ("Partition 0 start sector 0x%-08.8lX active %-02.2hX type %-02.2hX size %-08.8lX\n",
             pstart, pactive, ptype, psize);
#endif

        if (DFS_GetVolInfo(0, sector, pstart, &vi)) {
                printf("Error getting volume information\n");
                return -1;
        }
#if 0
        printf("Volume label '%-11.11s'\n", vi.label);
        printf
            ("%d sector/s per cluster, %d reserved sector/s, volume total %d sectors.\n",
             vi.secperclus, vi.reservedsecs, vi.numsecs);
        printf
            ("%d sectors per FAT, first FAT at sector #%d, root dir at #%d.\n",
             vi.secperfat, vi.fat1, vi.rootdir);
        printf
            ("(For FAT32, the root dir is a CLUSTER number, FAT12/16 it is a SECTOR number)\n");
        printf("%d root dir entries, data area commences at sector #%d.\n",
               vi.rootentries, vi.dataarea);
        printf("%d clusters (%d bytes) in data area, filesystem IDd as ",
               vi.numclusters, vi.numclusters * vi.secperclus * SECTOR_SIZE);
        if (vi.filesystem == FAT12)
                printf("FAT12.\n");
        else if (vi.filesystem == FAT16)
                printf("FAT16.\n");
        else if (vi.filesystem == FAT32)
                printf("FAT32.\n");
        else
                printf("[unknown]\n");
#endif

        di.scratch = sector;

        char *curpath = base_path;
        char *modify = strdup(curpath);

	if (DFS_OpenFile(&vi, base_path, DFS_READ, sector, &fi)) {
		printf("error opening file\n");
		return -1;
	}
	p = (void *) malloc(fi.filelen+512);
	memset(p, 0xaa, fi.filelen+512);

	DFS_ReadFile(&fi, sector, p, &i, fi.filelen);
	printf("read complete %d bytes (expected %d) pointer %d\n", i, fi.filelen, fi.pointer);

	{
	FILE *fp;
	fp=fopen(strrtok(base_path,"/"),"wb");
	fwrite(p, fi.filelen+512, 1, fp);
	fclose(fp);
	}
        free(data);

        return 0;
}

// Read the MS WinUSB Feature Descriptors, that are used on Windows 8 for automated driver installation
void
read_ms_winsub_feature_descriptors(libusb_device_handle * handle,
                                   uint8_t bRequest, int iface_number)
{
#define MAX_OS_FD_LENGTH 256
        int i, r;
        uint8_t os_desc[MAX_OS_FD_LENGTH];
        uint32_t length;
        void *le_type_punning_IS_fine;
        struct {
                char *desc;
                uint8_t recipient;
                uint16_t index;
                uint16_t header_size;
        } os_fd[2] = {
                {
                "Extended Compat ID", LIBUSB_RECIPIENT_DEVICE, 0x0004, 0x10},
                {
                "Extended Properties", LIBUSB_RECIPIENT_DEVICE, 0x0005,
                            0x0A}
                // NB: LIBUSB_RECIPIENT_INTERFACE should be used for the Extended Properties.
                // However, for Interface requests, the WinUSB DLL forces the low byte of wIndex
                // to the interface number, regardless of what you set it to, so we have to
                // fallback to Device and hope the firmware answers both equally.
                // See http://www.lvr.com/forum/index.php?topic=331
        };

        if (iface_number < 0)
                return;

        for (i = 0; i < 2; i++) {
                printf
                    ("\nReading %s OS Feature Descriptor (wIndex = 0x%04d):\n",
                     os_fd[i].desc, os_fd[i].index);

                // Read the header part
                r = libusb_control_transfer(handle,
                                            (uint8_t) (LIBUSB_ENDPOINT_IN |
                                                       LIBUSB_REQUEST_TYPE_VENDOR
                                                       | os_fd[i].recipient),
                                            bRequest,
                                            (uint16_t) (((iface_number) << 8) |
                                                        0x00), os_fd[i].index,
                                            os_desc, os_fd[i].header_size,
                                            1000);
                if (r < os_fd[i].header_size) {
                        return;
                }
                le_type_punning_IS_fine = (void *)os_desc;
                length = *((uint32_t *) le_type_punning_IS_fine);
                if (length > MAX_OS_FD_LENGTH) {
                        length = MAX_OS_FD_LENGTH;
                }
                // Read the full feature descriptor
                r = libusb_control_transfer(handle,
                                            (uint8_t) (LIBUSB_ENDPOINT_IN |
                                                       LIBUSB_REQUEST_TYPE_VENDOR
                                                       | os_fd[i].recipient),
                                            bRequest,
                                            (uint16_t) (((iface_number) << 8) |
                                                        0x00), os_fd[i].index,
                                            os_desc, (uint16_t) length, 1000);
                if (r < 0) {
                        perr("   Failed: %d", (r));
                        return;
                } else {
                        display_buffer_hex(os_desc, r);
                }
        }
}

int device_open(uint16_t vid, uint16_t pid)
{
        libusb_device_handle *handle;
        libusb_device *dev;
        uint8_t bus, port_path[8];
        struct libusb_config_descriptor *conf_desc;
        const struct libusb_endpoint_descriptor *endpoint;
        int i, j, k, r;
        int iface, nb_ifaces, first_iface = -1;
#if defined(__linux)
        // Attaching/detaching the kernel driver is only relevant for Linux
        int iface_detached = -1;
#endif
        struct libusb_device_descriptor dev_desc;
        char *speed_name[5] = { "Unknown", "1.5 Mbit/s (USB 1.0 LowSpeed)",
                "12 Mbit/s (USB 1.0 FullSpeed)",
                "480 Mbit/s (USB 2.0 HighSpeed)",
                "5000 Mbit/s (USB 3.0 SuperSpeed)"
        };
        char string[128];
        uint8_t string_index[3];  // indexes of the string descriptors
        uint8_t endpoint_in = 0, endpoint_out = 0;  // default IN and OUT endpoints

        handle = libusb_open_device_with_vid_pid(NULL, vid, pid);

        if (handle == NULL) {
                perr("  Failed.\n");
                return -1;
        }

        dev = libusb_get_device(handle);
        bus = libusb_get_bus_number(dev);
        r = libusb_get_port_path(NULL, dev, port_path, sizeof(port_path));

        r = libusb_get_device_speed(dev);
        if ((r < 0) || (r > 4))
                r = 0;

        CALL_CHECK(libusb_get_device_descriptor(dev, &dev_desc));

        // Copy the string descriptors for easier parsing
        string_index[0] = dev_desc.iManufacturer;
        string_index[1] = dev_desc.iProduct;
        string_index[2] = dev_desc.iSerialNumber;

        CALL_CHECK(libusb_get_config_descriptor(dev, 0, &conf_desc));
        nb_ifaces = conf_desc->bNumInterfaces;

        if (nb_ifaces > 0)
                first_iface =
                    conf_desc->usb_interface[0].altsetting[0].bInterfaceNumber;
        for (i = 0; i < nb_ifaces; i++) {

                for (j = 0; j < conf_desc->usb_interface[i].num_altsetting; j++) {

                        if ((conf_desc->usb_interface[i].altsetting[j].
                             bInterfaceClass == LIBUSB_CLASS_MASS_STORAGE)
                            &&
                            ((conf_desc->usb_interface[i].
                              altsetting[j].bInterfaceSubClass == 0x01)
                             || (conf_desc->usb_interface[i].
                                 altsetting[j].bInterfaceSubClass == 0x06))
                            && (conf_desc->usb_interface[i].
                                altsetting[j].bInterfaceProtocol == 0x50)) {
                                // Mass storage devices that can use basic SCSI commands
                                test_mode = USE_SCSI;
                        }
                        for (k = 0;
                             k <
                             conf_desc->usb_interface[i].altsetting[j].
                             bNumEndpoints; k++) {
                                endpoint =
                                    &conf_desc->usb_interface[i].altsetting[j].
                                    endpoint[k];

                                // Use the first bulk IN/OUT endpoints found as default for testing
                                if ((endpoint->bmAttributes &
                                     LIBUSB_TRANSFER_TYPE_MASK)
                                    == LIBUSB_TRANSFER_TYPE_BULK) {
                                        if (endpoint->bEndpointAddress &
                                            LIBUSB_ENDPOINT_IN) {
                                                if (!endpoint_in)
                                                        endpoint_in =
                                                            endpoint->
                                                            bEndpointAddress;
                                        } else {
                                                if (!endpoint_out)
                                                        endpoint_out =
                                                            endpoint->
                                                            bEndpointAddress;
                                        }
                                }
                        }
                }
        }
        libusb_free_config_descriptor(conf_desc);

        for (iface = 0; iface < nb_ifaces; iface++) {

                r = libusb_claim_interface(handle, iface);
#if defined(__linux)
                if ((r != LIBUSB_SUCCESS) && (iface == 0)) {
                        // Maybe we need to detach the driver
                        perr("   Failed. Trying to detach driver...\n");
                        libusb_detach_kernel_driver(handle, iface);
                        iface_detached = iface;
                        printf("   Claiming interface again...\n");
                        r = libusb_claim_interface(handle, iface);
                }
#endif
                if (r != LIBUSB_SUCCESS) {
                        perr("   Failed.\n");
                }
        }

        CALL_CHECK(mass_storage(handle, endpoint_in, endpoint_out));

        printf("\n");
        for (iface = 0; iface < nb_ifaces; iface++) {

                libusb_release_interface(handle, iface);
        }

#if defined(__linux)
        if (iface_detached >= 0) {

                libusb_attach_kernel_driver(handle, iface_detached);
        }
#endif

        libusb_close(handle);

        return 0;
}

int main(int argc, char **argv)
{
        bool show_help = false;
        bool debug_mode = false;
        const struct libusb_version *version;
        int j, r;
        size_t i, arglen;
        unsigned tmp_vid = 0, tmp_pid = 0;
        uint16_t endian_test = 0xBE00;

        // Default to generic, expecting VID:PID
        VID = 0;
        PID = 0;
        test_mode = USE_GENERIC;

        if (getenv("PICTURE_PATH"))
                base_path = getenv("PICTURE_PATH");

        if (((uint8_t *) & endian_test)[0] == 0xBE) {
                printf
                    ("Despite their natural superiority for end users, big endian\n"
                     "CPUs are not supported with this program, sorry.\n");
                return 0;
        }

        if (getenv("USB_DEVICE")) {
                if (sscanf_s(getenv("USB_DEVICE"), "%x:%x", &tmp_vid, &tmp_pid)
                    != 2) {
                        printf
                            ("   Please specify VID & PID as \"vid:pid\" in hexadecimal format in USB_DEVICE.\n");
                        return 1;
                }
        }

        VID = (uint16_t) tmp_vid;
        PID = (uint16_t) tmp_pid;

        if (argc != 2) {
                printf("usage: %s [file]\n", argv[0]);
                return 0;
        }

        base_path = argv[1];

        r = libusb_init(NULL);
        if (r < 0)
                return r;

        // Info = 3, Debug = 4
        libusb_set_debug(NULL, debug_mode ? 4 : 3);

        device_open(VID, PID);

        libusb_exit(NULL);

        return 0;
}
