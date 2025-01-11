/*******************************************************************************
#                                                                              #
#      MJPG-streamer allows to stream JPG frames from an input-plugin          #
#      to several output plugins                                               #
#                                                                              #
#      Copyright (C) 2008 Tom Stöveken                                         #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; version 2 of the License.                      #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/
#include "neatvnc.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>
#include <syslog.h>

#include <aml.h>
#include <signal.h>
#include <assert.h>
#include <pixman.h>

#include <jpeglib.h>
#include <libdrm/drm_fourcc.h>

#include "../../utils.h"
#include "../../mjpg_streamer.h"

#define OUTPUT_PLUGIN_NAME "VNC output plugin"

static pthread_t worker;
static pthread_t server;
static globals *pglobal;
static unsigned char *frame = NULL;
static int input_number = 0;

struct nvnc* vncserver = NULL;
struct nvnc_display* display = NULL;
struct pixman_region16 damage;

/******************************************************************************
Description.: print a help message
Input Value.: -
Return Value: -
******************************************************************************/
void help(void)
{
    fprintf(stderr, " ---------------------------------------------------------------\n" \
            " Help for output plugin..: "OUTPUT_PLUGIN_NAME"\n" \
            " ---------------------------------------------------------------\n");
}

/******************************************************************************
Description.: clean up allocated resources
Input Value.: unused argument
Return Value: -
******************************************************************************/
static void on_sigint()
{
	aml_exit(aml_get_default());
}

void worker_cleanup(void *arg)
{
    static unsigned char first_run = 1;

    if(!first_run) {
        DBG("already cleaned up resources\n");
        return;
    }

    first_run = 0;
    OPRINT("cleaning up resources allocated by worker thread\n");

    free(frame);
    //SDL_Quit();
}

typedef struct {
    struct jpeg_source_mgr pub;

    uint8_t *jpegdata;
    int jpegsize;
} my_source_mgr;

static void init_source(j_decompress_ptr cinfo)
{
    return;
}

static int fill_input_buffer(j_decompress_ptr cinfo)
{
    my_source_mgr * src = (my_source_mgr *) cinfo->src;

    src->pub.next_input_byte = src->jpegdata;
    src->pub.bytes_in_buffer = src->jpegsize;

    return TRUE;
}

static void skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
    my_source_mgr * src = (my_source_mgr *) cinfo->src;

    if(num_bytes > 0) {
        src->pub.next_input_byte += (size_t) num_bytes;
        src->pub.bytes_in_buffer -= (size_t) num_bytes;
    }
}

static void term_source(j_decompress_ptr cinfo)
{
    return;
}

static void jpeg_init_src(j_decompress_ptr cinfo, uint8_t *jpegdata, int jpegsize)
{
    my_source_mgr *src;

    if(cinfo->src == NULL) {  /* first time for this JPEG object? */
        cinfo->src = (struct jpeg_source_mgr *)(*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(my_source_mgr));
        src = (my_source_mgr *) cinfo->src;
    }

    src = (my_source_mgr *) cinfo->src;
    src->pub.init_source = init_source;
    src->pub.fill_input_buffer = fill_input_buffer;
    src->pub.skip_input_data = skip_input_data;
    src->pub.resync_to_restart = jpeg_resync_to_restart;
    src->pub.term_source = term_source;
    src->pub.bytes_in_buffer = 0; /* forces fill_input_buffer on first read */
    src->pub.next_input_byte = NULL; /* until buffer loaded */

    src->jpegdata = jpegdata;
    src->jpegsize = jpegsize;
}

static void my_error_exit(j_common_ptr cinfo)
{
    DBG("JPEG data contains an error\n");
}

static void my_error_output_message(j_common_ptr cinfo)
{
    DBG("JPEG data contains an error\n");
}

struct nvnc_fb* decompress_jpeg_2_fb(unsigned char *jpeg, int jpegsize)
{
    struct jpeg_decompress_struct cinfo;
    //JSAMPROW rowptr[1];
    struct jpeg_error_mgr jerr;

    /* create an error handler that does not terminate MJPEG-streamer */
    //DBG("decompress_jpeg_2_fb 1\n");
    cinfo.err = jpeg_std_error(&jerr);
    jerr.error_exit = my_error_exit;
    jerr.output_message = my_error_output_message;
    //DBG("decompress_jpeg_2_fb 2\n");
    /* create the decompressor structures */
    jpeg_create_decompress(&cinfo);

    /* initalize the structures of decompressor */
    jpeg_init_src(&cinfo, jpeg, jpegsize);
    //DBG("decompress_jpeg_2_fb 3\n");
    /* read the JPEG header data */
    if(jpeg_read_header(&cinfo, TRUE) < 0) {
        jpeg_destroy_decompress(&cinfo);
        DBG("could not read the header\n");
        //return 1;
        return NULL;
    }

    /*
     * I just expect RGB colored JPEGs, so the num_components must be three
     */
    //DBG("decompress_jpeg_2_fb 4\n");
    if(cinfo.num_components != 3) {
        jpeg_destroy_decompress(&cinfo);
        DBG("unsupported number of components (~colorspace)\n");
        //return 1;
        return NULL;
    }
    //DBG("decompress_jpeg_2_fb 5\n");
    /* just use RGB output and adjust decompression parameters */
    cinfo.out_color_space = JCS_RGB;
    cinfo.quantize_colors = FALSE;
    /* to scale the decompressed image, the fraction could be changed here */
    cinfo.scale_num   = 1;
    cinfo.scale_denom = 1;
    cinfo.dct_method = JDCT_FASTEST;
    cinfo.do_fancy_upsampling = FALSE;

    jpeg_calc_output_dimensions(&cinfo);

    /* store the image information */
    unsigned long width = cinfo.output_width;//图像宽度

    unsigned long height = cinfo.output_height;//图像高度

    unsigned short depth = cinfo.output_components;//图像深度

    struct nvnc_fb* fb = nvnc_fb_new(width, height, DRM_FORMAT_BGR888,
			width);
            
    assert(fb);
    //DBG("decompress_jpeg_2_fb 6\n");
    uint8_t* addr = nvnc_fb_get_addr(fb);

    /* start to decompress */
    if(jpeg_start_decompress(&cinfo) < 0) {
        jpeg_destroy_decompress(&cinfo);
        DBG("could not start decompression\n");
        return NULL;
    }
    //DBG("decompress_jpeg_2_fb 8\n");
    JSAMPROW rowptr[1];
	unsigned char *point = addr;
    rowptr[0] = (JSAMPROW)point;
    //DBG("decompress_jpeg_2_fb 8 A\n");
    while(cinfo.output_scanline < cinfo.output_height) {
        rowptr[0] = (JSAMPROW)(uint8_t *)point + cinfo.output_scanline * width * depth;
        if(jpeg_read_scanlines(&cinfo, rowptr, (JDIMENSION) 1) < 0) {
            jpeg_destroy_decompress(&cinfo);
            DBG("could not decompress this line\n");
            return NULL;
        }
    }
    //DBG("decompress_jpeg_2_fb 9\n");
    if(jpeg_finish_decompress(&cinfo) < 0) {
        jpeg_destroy_decompress(&cinfo);
        DBG("could not finish compression\n");
        return NULL;
    }
    
    /* all is done */
    jpeg_destroy_decompress(&cinfo);

    return fb;
}

/******************************************************************************
Description.: this is the main worker thread
              it loops forever, grabs a fresh frame, decompressed the JPEG
              and displays the decoded data using SDL
Input Value.:
Return Value:
******************************************************************************/
void *worker_thread(void *arg)
{
    int frame_size = 0;
    struct nvnc_fb* fb = NULL;

    /* just allocate a large buffer for the JPEGs */
    if((frame = malloc(4096 * 1024)) == NULL) {
        OPRINT("not enough memory for worker thread\n");
        exit(EXIT_FAILURE);
    }

    /* set cleanup handler to cleanup allocated resources */
    pthread_cleanup_push(worker_cleanup, NULL);

    while(!pglobal->stop) {
        DBG("waiting for fresh frame\n");
        pthread_mutex_lock(&pglobal->in[input_number].db);
        pthread_cond_wait(&pglobal->in[input_number].db_update, &pglobal->in[input_number].db);

        /* read buffer */
        frame_size = pglobal->in[input_number].size;
        memcpy(frame, pglobal->in[input_number].buf, frame_size);

        pthread_mutex_unlock(&pglobal->in[input_number].db);

        /* decompress the JPEG and store results in memory */
        //DBG("worker_thread 1\n");
        fb=decompress_jpeg_2_fb(frame, frame_size);
        if(fb==NULL){
            DBG("could not properly decompress JPEG data\n");
            continue;
        }
        //DBG("worker_thread 2\n");
        uint8_t* addr = nvnc_fb_get_addr(fb);
        pixman_region_init_rect(&damage, 0, 0, nvnc_fb_get_width(fb),
	    nvnc_fb_get_height(fb));
	    nvnc_display_feed_buffer(display, fb, &damage);
	    pixman_region_fini(&damage);
        nvnc_fb_unref(fb);
        DBG("worker_thread 3\n");
    }

    pthread_cleanup_pop(1);


    return NULL;
}

void *server_thread(void *arg)
{
    struct aml* aml = aml_new();
	aml_set_default(aml);

	vncserver = nvnc_open("0.0.0.0", 5900);
	assert(vncserver);

	display = nvnc_display_new(0, 0);
	assert(display);

	nvnc_add_display(vncserver, display);
	nvnc_set_name(vncserver, "Test Display");
	//nvnc_set_pointer_fn(server, on_pointer_event);
	nvnc_set_userdata(vncserver, &display, NULL);
	

	struct aml_signal* sig = aml_signal_new(SIGINT, on_sigint, NULL, NULL);
	aml_start(aml_get_default(), sig);
	aml_unref(sig);

	aml_run(aml);

	nvnc_close(vncserver);
	nvnc_display_unref(display);
	aml_unref(aml);
    return NULL;
}
/*** plugin interface functions ***/
/******************************************************************************
Description.: this function is called first, in order to initialise
              this plugin and pass a parameter string
Input Value.: parameters
Return Value: 0 if everything is ok, non-zero otherwise
******************************************************************************/
int output_init(output_parameter *param)
{
    int i;

    param->argv[0] = OUTPUT_PLUGIN_NAME;

    /* show all parameters for DBG purposes */
    for(i = 0; i < param->argc; i++) {
        DBG("argv[%d]=%s\n", i, param->argv[i]);
    }

    reset_getopt();
    while(1) {
        int option_index = 0, c = 0;
        static struct option long_options[] = {
            {"h", no_argument, 0, 0
            },
            {"help", no_argument, 0, 0},
            {"i", required_argument, 0, 0},
            {"input", required_argument, 0, 0},
            {0, 0, 0, 0}
        };

        c = getopt_long_only(param->argc, param->argv, "", long_options, &option_index);

        /* no more options to parse */
        if(c == -1) break;

        /* unrecognized option */
        if(c == '?') {
            help();
            return 1;
        }

        switch(option_index) {
            /* h, help */
        case 0:
        case 1:
            DBG("case 0,1\n");
            help();
            return 1;
            break;
            /* i, input */
        case 2:
        case 3:
            DBG("case 2,3\n");
            input_number = atoi(optarg);
            break;
        }
    }

    pglobal = param->global;
    if(!(input_number < pglobal->incnt)) {
        OPRINT("ERROR: the %d input_plugin number is too much only %d plugins loaded\n", input_number, pglobal->incnt);
        return 1;
    }
    OPRINT("input plugin.....: %d: %s\n", input_number, pglobal->in[input_number].plugin);

    return 0;
}

/******************************************************************************
Description.: calling this function stops the worker thread
Input Value.: -
Return Value: always 0
******************************************************************************/
int output_stop(int id)
{
    DBG("will cancel worker thread\n");
    pthread_cancel(worker);
    pthread_cancel(server);
    aml_exit(aml_get_default());
    return 0;
}

/******************************************************************************
Description.: calling this function creates and starts the worker thread
Input Value.: -
Return Value: always 0
******************************************************************************/
int output_run(int id)
{
    DBG("launching server thread\n");
    pthread_create(&server, 0, server_thread, NULL);
    pthread_detach(server);
    DBG("launching worker thread\n");
    pthread_create(&worker, 0, worker_thread, NULL);
    pthread_detach(worker);
    return 0;
}

int output_cmd()
{


}

