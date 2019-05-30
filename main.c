#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <linux/types.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#define __packed __attribute__((packed))
/* HCI data types */                                                 
#define HCI_COMMAND_PKT     0x01            
#define HCI_ACLDATA_PKT     0x02
#define HCI_SCODATA_PKT     0x03
#define HCI_EVENT_PKT       0x04
#define HCI_DIAG_PKT        0xf0
#define HCI_VENDOR_PKT      0xff

struct hci_command_hdr {
	__le16  opcode;     /* OCF & OGF */         
	__u8    plen;                                     
} __packed;     

struct hci_event_hdr {                      
	__u8    evt;                                      
	__u8    plen;
} __packed;

struct hci_acl_hdr {
	__le16  handle;     /* Handle & Flags(PB, BC) */
	__le16  dlen;  
} __packed;

struct hci_sco_hdr {                                    
	__le16  handle;
	__u8    dlen;
} __packed;

static int open_channel(uint16_t index)
{
	int fd;
	int on = 1;
	struct sockaddr_hci addr;

	printf("Opening user channel for hci%u\n", index);

	fd = socket(PF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC, BTPROTO_HCI);
	if (fd < 0) {
		perror("Failed to open Bluetooth socket");
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.hci_family = AF_BLUETOOTH;
	addr.hci_dev = index;
	addr.hci_channel = HCI_CHANNEL_USER;

	if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
		close(fd);
		perror("setsockopt");
		return -1;
	}

	if (bind(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
		close(fd);
		perror("Failed to bind Bluetooth socket");
		return -1;
	}

	return fd;
}

static void hci_event_packet(struct hci_event_hdr *hdr)
{
	switch (hdr->evt) {
	default:
		printf("unhandle evt: %x, %d\n", hdr->evt, hdr->plen);
	break;
	}
}

static void hci_acldata_packet(struct hci_acl_hdr *hdr)
{
}

static void hci_scodata_packet(struct hci_sco_hdr *hdr)
{
}

int main(void)
{
	int fd;
	int rsize;
	uint8_t buf[1024];

	fd = open_channel(0);
	if(fd < 0)
		return EXIT_FAILURE;

	while (true) {
		rsize = read(fd, buf, sizeof(buf));

		if(rsize < 0)
			return EXIT_FAILURE;

		switch (buf[0]) {
		case HCI_EVENT_PKT:
			if(rsize < sizeof(struct hci_event_hdr))
				return EXIT_FAILURE;
			hci_event_packet((struct hci_event_hdr *)(buf + 1));
		break;
		case HCI_ACLDATA_PKT:
			if(rsize < sizeof(struct hci_acl_hdr))
				return EXIT_FAILURE;
			hci_acldata_packet((struct hci_acl_hdr*)(buf + 1));
		break;
		case HCI_SCODATA_PKT:
			if(rsize < sizeof(struct hci_sco_hdr))
				return EXIT_FAILURE;
			hci_scodata_packet((struct hci_sco_hdr*)(buf + 1));
		break;

		default:
		break;
		}
	}

	close(fd);

	return EXIT_SUCCESS;
}
