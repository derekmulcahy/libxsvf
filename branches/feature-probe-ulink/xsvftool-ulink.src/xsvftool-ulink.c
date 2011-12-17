/*
 *  xsvftool-ulink - An (X)SVF player for the Keil ULINK JTAG Probe
 *
 *  Copyright (C) 2011  Clifford Wolf <clifford@clifford.at>
 *  
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "libxsvf.h"

#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <usb.h>

#define DEFAULT_VENDOR_ID 0xC251
#define DEFAULT_PRODUCT_ID 0x2710

#define UNUSED __attribute__((unused))

int idVendor  = DEFAULT_VENDOR_ID;
int idProduct = DEFAULT_PRODUCT_ID;

struct udata_s
{
	FILE *f;
	int verbose;
	int retval_i;
	int retval[256];
	usb_dev_handle *usbdev;

	uint8_t cmdidx;
	uint8_t cmdbuf[64];
	uint8_t retbuf_expect_set[16];
	uint8_t retbuf_expect_clr[16];
	uint8_t retbuf_toretval[16];
	int error, last_tdo;

	uint8_t backlog_len;
	uint8_t backlog_expect_set[16];
	uint8_t backlog_expect_clr[16];
	uint8_t backlog_toretval[16];

	int last_tdi, last_tms, last_tck, last_trst;
};

int usb_send_chunk(usb_dev_handle *dh, int ep, const void *data, int len)
{
	int ret;
#if 0
	int i;
	fprintf(stderr, "<ep%d:%4d bytes out>", ep, len);
	for (i = 0; i < len; i++)
		fprintf(stderr, " %02x", ((unsigned char *)data)[i]);
	fprintf(stderr, "\n");
#endif
retry_write:
	ret = usb_bulk_write(dh, ep, data, len, 1000);
	if (ret == -ETIMEDOUT) {
		fprintf(stderr, "usb_recv_chunk: usb write timeout -> retry\n");
		goto retry_write;
	}
	if (ret != len)
		fprintf(stderr, "usb_send_chunk: write of %d bytes to ep %d returned %d: %s\n", len, ep, ret, ret >= 0 ? "NO ERROR" : usb_strerror());
	return ret == len ? 0 : -1;
}

int usb_recv_chunk(usb_dev_handle *dh, int ep, void *data, int len, int *ret_len)
{
	int ret;
retry_read:
	ret = usb_bulk_read(dh, ep, data, len, 1000);
	if (ret == -ETIMEDOUT) {
		fprintf(stderr, "usb_recv_chunk: usb read timeout -> retry\n");
		goto retry_read;
	}
	if (ret > 0 && ret_len != NULL)
		len = *ret_len = ret;
	if (ret != len)
		fprintf(stderr, "usb_recv_chunk: read of %d bytes from ep %d returned %d: %s\n", len, ep, ret, ret >= 0 ? "NO ERROR" : usb_strerror());
#if 0
	int i;
	fprintf(stderr, "<ep%d:%4d bytes in>", ep, len);
	for (i = 0; i < len; i++)
		fprintf(stderr, " %02x", ((unsigned char *)data)[i]);
	fprintf(stderr, "\n");
#endif
	return ret == len ? 0 : -1;
}

static void xfer_response(struct udata_s *u, uint8_t *expect_set UNUSED, uint8_t *expect_clr UNUSED, uint8_t *toretval, uint8_t len)
{
	uint8_t response[len], i, j;
	usb_recv_chunk(u->usbdev, 2, response, len, NULL);

	for (i = 0; i < len; i++)
	{
		printf("> %2d: %02x %02x %02x\n", i, response[i], expect_set[i], expect_clr[i]);
		if ((response[i] & expect_set[i]) != expect_set[i])
			u->error = 1;
		if ((~response[i] & expect_clr[i]) != expect_clr[i])
			u->error = 1;
		for (j = 0; toretval[i] && j < 8; j++)
			if ((toretval[i] & (1 << j)) != 0)
				u->retval[u->retval_i++] = (response[i] & (1 << j)) != 0;
		u->last_tdo = (response[i] & 0x80) != 0;
	}
}

static void queue(struct udata_s *u, int tdi, int tms, int tck, int trst, int sync);

static void xfer(struct udata_s *u, int sync)
{
	while (u->cmdidx % 8 != 0)
		queue(u, -1, -1, 1, -1, -1);
	u->cmdbuf[0] = 0x06;
	u->cmdbuf[1] = u->cmdidx / 8;

	if (u->backlog_len) {
		xfer_response(u, u->backlog_expect_set, u->backlog_expect_clr,
				u->backlog_toretval, u->backlog_len);
		u->backlog_len = 0;
	}

	// For probes with double buffering on EP2:
	// Move this block before xfer_response backlog block above
	// for improved speed. But without double buffering this would
	// result in a deadlock. So we don't do it atm..
	usb_send_chunk(u->usbdev, 2, u->cmdbuf, u->cmdbuf[1]*4 + 2);

	u->last_tdo = -1;
	if (sync) {
		xfer_response(u, u->retbuf_expect_set, u->retbuf_expect_clr,
				u->retbuf_toretval, u->cmdbuf[1]);
	} else {
		u->backlog_len = u->cmdbuf[1];
		memcpy(u->backlog_expect_set, u->retbuf_expect_set, 16);
		memcpy(u->backlog_expect_clr, u->retbuf_expect_clr, 16);
		memcpy(u->backlog_toretval, u->retbuf_toretval, 16);
	}

	u->cmdidx = 0;
}

static void queue(struct udata_s *u, int tdi, int tms, int tck, int trst, int sync)
{
	if (u->cmdidx == 0) {
		memset(u->cmdbuf, 0, 64);
		memset(u->retbuf_expect_set, 0, 16);
		memset(u->retbuf_expect_clr, 0, 16);
		memset(u->retbuf_toretval, 0, 16);
	}

	if (tdi < 0)
		tdi = u->last_tdi;
	if (tms < 0)
		tms = u->last_tms;
	if (tck < 0)
		tck = u->last_tck;
	if (trst < 0)
		trst = u->last_trst;

	int command = 0;
	if (tdi)
		command |= 0x01;
	if (tms)
		command |= 0x02;
	if (tck)
		command |= 0x04;
	if (trst)
		command |= 0x08;

	uint8_t cmdoff = (u->cmdidx++ >> 1) + 2;
	if (u->cmdidx % 2 == 0)
		u->cmdbuf[cmdoff] |= command;
	else
		u->cmdbuf[cmdoff] |= command << 4;

	u->last_tdi = tdi;
	u->last_tms = tms;
	u->last_tck = tck;
	u->last_trst = trst;

	if (sync < 0)
		return;

	if (u->cmdidx >= 58*2 || sync)
		xfer(u, sync);
}

static int h_setup(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;

	u->cmdidx = 0;
	u->error = 0;
	u->backlog_len = 0;
	queue(u, 1, 1, 1, 1, 0);

	if (u->verbose >= 2) {
		fprintf(stderr, "[SETUP]\n");
		fflush(stderr);
	}

	struct usb_bus *b;
	struct usb_device *d;
	for (b = usb_get_busses(); b; b = b->next) {
		for (d = b->devices; d; d = d->next) {
			if ((d->descriptor.idVendor == idVendor) && (d->descriptor.idProduct == idProduct)) {
				u->usbdev = usb_open(d);
				if (usb_claim_interface(u->usbdev, 0) < 0) {
					usb_close(u->usbdev);
					continue;
				}
				return 0;
			}
		}
	}

	fprintf(stderr, "Couldn't open ULINK device with VID = 0x%04X and PID = 0x%04X.\n", idVendor, idProduct);
	return -1;
}

static int h_shutdown(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 2) {
		fprintf(stderr, "[SHUTDOWN]\n");
		fflush(stderr);
	}
	usb_release_interface(u->usbdev, 0);
	usb_close(u->usbdev);
	return 0;
}

static void h_udelay(struct libxsvf_host *h, long usecs, int tms, long num_tck)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 3) {
		fprintf(stderr, "[DELAY:%ld, TMS:%d, NUM_TCK:%ld]\n", usecs, tms, num_tck);
		fflush(stderr);
	}
	if (num_tck > 0) {
		struct timeval tv1, tv2;
		gettimeofday(&tv1, NULL);
		queue(u, -1, tms, 1, -1, 0);
		while (num_tck > 0) {
			queue(u, -1, -1, 0, -1, 0);
			num_tck--;
		}
		gettimeofday(&tv2, NULL);
		if (tv2.tv_sec > tv1.tv_sec) {
			usecs -= (1000000 - tv1.tv_usec) + (tv2.tv_sec - tv1.tv_sec - 1) * 1000000;
			tv1.tv_usec = 0;
		}
		usecs -= tv2.tv_usec - tv1.tv_usec;
		if (u->verbose >= 3) {
			fprintf(stderr, "[DELAY_AFTER_TCK:%ld]\n", usecs > 0 ? usecs : 0);
			fflush(stderr);
		}
	}
	if (usecs > 0) {
		usleep(usecs);
	}
}

static int h_getbyte(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	return fgetc(u->f);
}

static int h_sync(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	if (u->cmdidx == 0)
		return 0;
	xfer(u, 1);
	if (u->error) {
		u->error = 0;
		return -1;
	}
	return 0;
}

static int h_pulse_tck(struct libxsvf_host *h, int tms, int tdi, int tdo, int rmask, int sync)
{
	struct udata_s *u = h->user_data;
	int rc = 0;

	int cmdidx = u->cmdidx;
	queue(u, tdi, tms, 0, -1, -1);

	if (tdo == 1)
		u->retbuf_expect_set[cmdidx / 8] |= 1 << (cmdidx % 8);
	if (tdo == 0)
		u->retbuf_expect_clr[cmdidx / 8] |= 1 << (cmdidx % 8);
	if (rmask)
		u->retbuf_toretval[cmdidx / 8] |= 1 << (cmdidx % 8);

	if (u->cmdidx >= 58*2 || sync)
		xfer(u, sync);

	rc = tdo < 0 ? 1 : tdo;
	if (sync || (tdo >= 0 && u->error)) {
		if (u->error) {
			u->error = 0;
			rc = -1;
		} else
			rc = u->last_tdo;
	}

	if (u->verbose >= 4) {
		fprintf(stderr, "[TMS:%d, TDI:%d, TDO_ARG:%d, TDO_LINE:%d, RMASK:%d, RC:%d]\n", tms, tdi, tdo, u->last_tdo, rmask, rc);
	}

	return rc;
}

static void h_set_trst(struct libxsvf_host *h, int v)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 4) {
		fprintf(stderr, "[TRST:%d]\n", v);
	}
	queue(u, -1, -1, -1, v, 0);
}

static int h_set_frequency(struct libxsvf_host *h UNUSED, int v)
{
	fprintf(stderr, "WARNING: Setting JTAG clock frequency to %d ignored!\n", v);
	return 0;
}

static void h_report_tapstate(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 3) {
		fprintf(stderr, "[%s]\n", libxsvf_state2str(h->tap_state));
	}
}

static void h_report_device(struct libxsvf_host *h UNUSED, unsigned long idcode)
{
	// struct udata_s *u = h->user_data;
	printf("idcode=0x%08lx, revision=0x%01lx, part=0x%04lx, manufactor=0x%03lx\n", idcode,
			(idcode >> 28) & 0xf, (idcode >> 12) & 0xffff, (idcode >> 1) & 0x7ff);
}

static void h_report_status(struct libxsvf_host *h, const char *message)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 2) {
		fprintf(stderr, "[STATUS] %s\n", message);
	}
}

static void h_report_error(struct libxsvf_host *h UNUSED, const char *file, int line, const char *message)
{
	fprintf(stderr, "[%s:%d] %s\n", file, line, message);
}

static void *h_realloc(struct libxsvf_host *h, void *ptr, int size, enum libxsvf_mem which)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 3) {
		fprintf(stderr, "[REALLOC:%s:%d]\n", libxsvf_mem2str(which), size);
	}
	return realloc(ptr, size);
}

static struct udata_s u;

static struct libxsvf_host h = {
	.udelay = h_udelay,
	.setup = h_setup,
	.shutdown = h_shutdown,
	.getbyte = h_getbyte,
	.sync = h_sync,
	.pulse_tck = h_pulse_tck,
	.set_trst = h_set_trst,
	.set_frequency = h_set_frequency,
	.report_tapstate = h_report_tapstate,
	.report_device = h_report_device,
	.report_status = h_report_status,
	.report_error = h_report_error,
	.realloc = h_realloc,
	.user_data = &u
};

const char *progname;

static void copyleft()
{
	static int already_printed = 0;
	if (already_printed)
		return;
	fprintf(stderr, "xsvftool-ulink, Lib(X)SVF based host for Keil ULINK probes.\n");
	fprintf(stderr, "Copyright (C) 2011  Clifford Wolf <clifford@clifford.at>\n");
	fprintf(stderr, "Lib(X)SVF is free software licensed under the ISC license.\n");
	already_printed = 1;
}

static void help()
{
	copyleft();
	fprintf(stderr, "\n");
	fprintf(stderr, "Usage: %s [ -r funcname ] [ -v ... ] [ -L | -B ] { -s svf-file | -x xsvf-file | -c } ...\n", progname);
	fprintf(stderr, "\n");
	fprintf(stderr, "   -v, -vv, -vvv, -vvvv\n");
	fprintf(stderr, "          Verbose, more verbose and even more verbose\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -L, -B\n");
	fprintf(stderr, "          Print RMASK bits as hex value (little or big endian)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -s svf-file\n");
	fprintf(stderr, "          Play the specified SVF file\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -x xsvf-file\n");
	fprintf(stderr, "          Play the specified XSVF file\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -c\n");
	fprintf(stderr, "          List devices in JTAG chain\n");
	fprintf(stderr, "\n");
	exit(1);
}

int main(int argc, char **argv)
{
	int rc = 0;
	int gotaction = 0;
	int hex_mode = 0;
	int opt, i, j;

	usb_init();
	usb_find_busses();
	usb_find_devices();

	progname = argc >= 1 ? argv[0] : "xvsftool";
	while ((opt = getopt(argc, argv, "vLBx:s:c")) != -1)
	{
		switch (opt)
		{
		case 'v':
			copyleft();
			u.verbose++;
			break;
		case 'x':
		case 's':
			gotaction = 1;
			if (u.verbose)
				fprintf(stderr, "Playing %s file `%s'.\n", opt == 's' ? "SVF" : "XSVF", optarg);
			if (!strcmp(optarg, "-"))
				u.f = stdin;
			else
				u.f = fopen(optarg, "rb");
			if (u.f == NULL) {
				fprintf(stderr, "Can't open %s file `%s': %s\n", opt == 's' ? "SVF" : "XSVF", optarg, strerror(errno));
				rc = 1;
				break;
			}
			if (libxsvf_play(&h, opt == 's' ? LIBXSVF_MODE_SVF : LIBXSVF_MODE_XSVF) < 0) {
				fprintf(stderr, "Error while playing %s file `%s'.\n", opt == 's' ? "SVF" : "XSVF", optarg);
				rc = 1;
			}
			if (strcmp(optarg, "-"))
				fclose(u.f);
			break;
		case 'c':
			gotaction = 1;
			if (libxsvf_play(&h, LIBXSVF_MODE_SCAN) < 0) {
				fprintf(stderr, "Error while scanning JTAG chain.\n");
				rc = 1;
			}
			break;
		case 'L':
			hex_mode = 1;
			break;
		case 'B':
			hex_mode = 2;
			break;
		default:
			help();
			break;
		}
	}

	if (!gotaction)
		help();

	if (u.retval_i) {
		if (hex_mode) {
			printf("0x");
			for (i=0; i < u.retval_i; i+=4) {
				int val = 0;
				for (j=i; j<i+4; j++)
					val = val << 1 | u.retval[hex_mode > 1 ? j : u.retval_i - j - 1];
				printf("%x", val);
			}
		} else {
			printf("%d rmask bits:", u.retval_i);
			for (i=0; i < u.retval_i; i++)
				printf(" %d", u.retval[i]);
		}
		printf("\n");
	}

	return rc;
}

