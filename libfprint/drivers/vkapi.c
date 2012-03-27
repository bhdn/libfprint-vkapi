/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; version
 * 2.1 of the License.
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

#define FP_COMPONENT "vkapi"

#include <errno.h>
#include <string.h>
#include <dlfcn.h>
#include <assert.h>

#include <glib.h>
#include <libusb.h>

#include <fp_internal.h>

#define VKAPI_SONAME "vkapi.so"

#define VKX_RESULT_FAIL 0
#define VKX_RESULT_SUCCESS 1
#define VKX_RESULT_MATCHED VKX_RESULT_SUCCESS
#define VKX_RESULT_NOT_MATCHED VKX_RESULT_FAIL
#define VKX_RESULT_ADD_DUPLICATE 4
#define VKX_RESULT_DUPLICATE_FEATURE 5
#define VKX_RESULT_NOT_CONNECTED 6
#define VKX_RESULT_LIMITED 7
#define VKX_RESULT_ENROLL_FAIL -1001

#define OP_TYPE_ENROLL             101
#define OP_TYPE_VERIFY             102
#define OP_TYPE_NAVIGATION         103

#define  STATUS_SENSOR_OPEN        1
#define  STATUS_SENSOR_CLOSE       2
#define  STATUS_IMAGE_FETCH        3
#define  STATUS_IMAGE_READY        4
#define  STATUS_IMAGE_BAD          5
#define  STATUS_FEATURE_LOW        6
#define  STATUS_OPERATION_BEGIN    7
#define  STATUS_OPERATION_END      8
#define  STATUS_IMAGE_FETCHING     9
#define  STATUS_FINGER_DETECTED   10
#define  STATUS_FINGER_REMOVED    11
#define  STATUS_SWIPE_TOO_FAST    12
#define  STATUS_SWIPE_TOO_SLOW    13
#define  STATUS_SWIPE_TOO_SHORT   14
#define  STATUS_SWIPE_TOO_SKEWED  15
#define  STATUS_SWIPE_TOO_LEFT    16
#define  STATUS_SWIPE_TOO_RIGHT   17
#define  STATUS_SWIPE_TOO_WET     25
#define  STATUS_SWIPE_TOO_DRY     26
#define  STATUS_SENSOR_UNPLUG     18
#define  STATUS_USER_TOO_FAR      19
#define  STATUS_USER_TOO_CLOSE    20
#define  STATUS_LUX_TOO_LOWER     21
#define  STATUS_LUX_TOO_HIGHER    22
#define  STATUS_FINGER_TOUCH      23
#define  STATUS_FINGER_REMOVE     24
#define  STATUS_SENSOR_TIMEOUT    27
#define  STATUS_USER_ABORT        28

#define  ERROR_ENROLL_FAIL        -1001
#define  ERROR_ENROLL_DUPLICATED  -1009
#define  E6 1000000
#define  MAX_BASE64_LENGTH 3072

typedef void (*t_onenrollprogress)(int pos);
typedef void (*t_onstatus)(int status);
typedef void (*t_onerror)(int error, int *retry);
typedef void (*t_ongetimage)(int width, int height, unsigned char *img, int qty, int *retry);
typedef void (*t_ongettemplate)(unsigned char *feature, int size, int *retry);
typedef void (*t_ongetBase64template)(char *feat_b64, int size, int *retry);
typedef void (*t_onnavigation)(int dx, int dy, int touch);

static int (*fvkx_connect) (void) = NULL;
static int (*fvkx_dis_connect) (void) = NULL;
static int (*fvkx_abort) (void) = NULL;
static int (*fvkx_capture_verify_template) (void) = NULL;
static int (*fvkx_capture_enroll_template) (void) = NULL;
static int (*fvkx_set_enroll_progress) (t_onenrollprogress fn) = NULL;
static int (*fvkx_set_status_callback) (t_onstatus fn) = NULL;
static int (*fvkx_set_error_callback) (t_onerror fn) = NULL;
static int (*fvkx_set_template_get_callback) (t_ongettemplate fn) = NULL;
static int (*fvkx_set_Base64_template_get_callback) (t_ongetBase64template fn) = NULL;
static int (*fvkx_set_image_get_callback) (t_ongetimage fn) = NULL;
static int (*fvkx_set_navigation_callback) (t_onnavigation fn) = NULL;
static int (*fvkx_get_operation_type) (int *pt) = NULL;
static int (*fvkx_set_device_type) (int n) = NULL;
static int (*fvkx_verify) (unsigned char* enroll, unsigned char* feature,
	int *score, int isBase64) = NULL;
static int (*fvkx_get_gui_need_lock) (void) = NULL;

/* vkapi does not allow setting user pointers when registering callbacks,
 * requiring us to make the driver state global */
static struct fpi_ssm *global_enroll_ssm = NULL;
static struct fpi_ssm *global_verify_ssm = NULL;
static struct fp_dev *global_dev = NULL;

static int load_api(void)
{
	void *handle = dlopen(VKAPI_SONAME, RTLD_LAZY);
	if (!handle) {
		fp_err("failed to load %s: %s", VKAPI_SONAME, dlerror());
		return 1;
	}

	fvkx_connect = dlsym(handle, "vkx_connect");
	fvkx_dis_connect = dlsym(handle, "vkx_dis_connect");
	fvkx_abort = dlsym(handle, "vkx_abort");
	fvkx_capture_verify_template =
	    dlsym(handle, "vkx_capture_verify_template");
	fvkx_capture_enroll_template =
	    dlsym(handle, "vkx_capture_enroll_template");
	fvkx_set_enroll_progress = dlsym(handle, "vkx_set_enroll_progress");
	fvkx_set_status_callback = dlsym(handle, "vkx_set_status_callback");
	fvkx_set_error_callback = dlsym(handle, "vkx_set_error_callback");
	fvkx_set_template_get_callback =
	    dlsym(handle, "vkx_set_template_get_callback");
	fvkx_set_Base64_template_get_callback =
	    dlsym(handle, "vkx_set_Base64_template_get_callback");
	fvkx_set_image_get_callback =
	    dlsym(handle, "vkx_set_image_get_callback");
	fvkx_get_operation_type = dlsym(handle, "vkx_get_operation_type");
	fvkx_set_device_type = dlsym(handle, "vkx_set_device_type");
	fvkx_verify = dlsym(handle, "vkx_verify");
	fvkx_get_gui_need_lock = dlsym(handle, "vkx_get_gui_need_lock");

	return 0;
}

static void callback_getimage(int width, int height, unsigned char *raw,
		unsigned long imgQty, int *reRun)
{
	fp_dbg("getimage invoked");
	return;
}

static void callback_gettemplate(unsigned char *feat, int len, int *reRun)
{
	assert(global_dev);

	if (global_enroll_ssm) {
		struct fp_print_data *fdata = fpi_print_data_new(global_dev, len);
		memcpy(fdata->data, feat, len);

		fpi_drvcb_enroll_stage_completed(global_dev, FP_ENROLL_COMPLETE, fdata, NULL);
		fpi_ssm_mark_completed(global_enroll_ssm);
	} else if (global_verify_ssm) {
		int fp_r;
		int vk_r;
		int score;

		vk_r = fvkx_verify(feat, global_dev->verify_data->data, &score, 0);
		if (vk_r == VKX_RESULT_MATCHED)
			fp_r = FP_VERIFY_MATCH;
		else
			fp_r = FP_VERIFY_NO_MATCH;

		fpi_drvcb_report_verify_result(global_dev, fp_r, NULL);
		fpi_ssm_mark_completed(global_verify_ssm);
	} else
		fp_err("_gettemplate without a ssm set??\n");
}

static void callback_onstatus(int status)
{
	switch (status) {
	case STATUS_SENSOR_OPEN:
		fp_dbg("STATUS_SENSOR_OPEN\n");
		break;
	case STATUS_SENSOR_CLOSE:
		fp_dbg("STATUS_SENSOR_CLOSE\n");
		break;
	case STATUS_IMAGE_FETCH:
		fp_dbg("STATUS_IMAGE_FETCH\n");
		break;
	case STATUS_IMAGE_READY:
		fp_dbg("STATUS_IMAGE_READY\n");
		break;
	case STATUS_IMAGE_BAD:
		fp_dbg("STATUS_IMAGE_BAD\n");
		break;
	case STATUS_FEATURE_LOW:
		fp_dbg("STATUS_FEATURE_LOW\n");
		break;
	case STATUS_OPERATION_BEGIN:
		fp_dbg("STATUS_OPERATION_BEGIN\n");
		if (global_enroll_ssm) {
			fpi_drvcb_enroll_started(global_dev, 0);
			fpi_ssm_next_state(global_enroll_ssm);
		} else if (global_verify_ssm) {
			fpi_drvcb_verify_started(global_dev, 0);
			fpi_ssm_next_state(global_verify_ssm);
		} else
			fp_dbg("OPERATION_BEGIN, but without an enroll ssm!\n");
		break;
	case STATUS_OPERATION_END:
		fp_dbg("STATUS_OPERATION_END\n");
		break;
	case STATUS_IMAGE_FETCHING:
		fp_dbg("STATUS_IMAGE_FETCHING\n");
		break;
	case STATUS_SENSOR_TIMEOUT:
		fp_dbg("STATUS_SENSOR_TIMEOUT\n");
		break;
	case STATUS_USER_ABORT:
		fp_dbg("STATUS_USER_ABORT\n");
		break;
	}
}

static void callback_onerror(int error, int *retry)
{
	switch (error) {
	case VKX_RESULT_NOT_CONNECTED:
		fp_dbg("VKX_RESULT_NOT_CONNECTED\n");
		break;
	case VKX_RESULT_ENROLL_FAIL:
		fp_dbg("VKX_RESULT_ENROLL_FAIL\n");
		break;
	}
}

static void callback_enroll_progress(int pos)
{
	/* FIXME failed stages may result in pos > 3 without having
	 * completed the enroll process! */
	if (global_enroll_ssm && pos < 3) {
		fpi_drvcb_enroll_stage_completed(global_dev,
				FP_ENROLL_PASS, NULL, NULL);
	}
}

static void set_callbacks(void)
{
	fvkx_set_image_get_callback((t_ongetimage) callback_getimage);
	fvkx_set_template_get_callback((t_ongettemplate) callback_gettemplate);
	fvkx_set_status_callback(callback_onstatus);
	fvkx_set_error_callback(callback_onerror);
	fvkx_set_enroll_progress(callback_enroll_progress);
}

static int dev_init(struct fp_dev *dev, unsigned long driver_data)
{
	int r = load_api();
	if (r != 0)
		return r;

	set_callbacks();

	r = fvkx_connect();
	fp_dbg("vkx_connect() = %d\n", r);
	if (r != 1)
		return -1;

	dev->nr_enroll_stages = 3;

	/* FIXME wait for STATUS_SENSOR_OPEN before marking as complete */
	fpi_drvcb_open_complete(dev, 0);
	return 0;
}

enum verify_start_sm_states {
	VERIFY_INITSM = 0,
	VERIFY_INIT,
	VERIFY_IMAGE_FETCH,
	VERIFY_START_NUM_STATES,
};

static void verify_start_sm_run_state(struct fpi_ssm *ssm)
{
	struct fp_dev *dev = ssm->dev;

	switch (ssm->cur_state) {
	case VERIFY_INITSM:
		fpi_ssm_next_state(ssm);
		break;
	case VERIFY_INIT:
		fvkx_capture_verify_template();
		break;
	case VERIFY_IMAGE_FETCH:
		break;
	}
}

enum enroll_start_sm_states {
	ENROLL_INITSM = 0,
	ENROLL_INIT,
	ENROLL_IMAGE_FETCH,
	ENROLL_START_NUM_STATES,
};

static void enroll_start_sm_run_state(struct fpi_ssm *ssm)
{
	struct fp_dev *dev = ssm->dev;

	switch (ssm->cur_state) {
	case ENROLL_INITSM:
		fpi_ssm_next_state(ssm);
		break;
	case ENROLL_INIT:
		fvkx_capture_enroll_template();
		break;
	case ENROLL_IMAGE_FETCH:
		break;
	}
}

static void enroll_done(struct fpi_ssm *ssm)
{
	struct fp_dev *dev = ssm->dev;

	fpi_ssm_free(ssm);
}

static int enroll_start(struct fp_dev *dev)
{
	global_enroll_ssm = fpi_ssm_new(dev, enroll_start_sm_run_state,
			ENROLL_START_NUM_STATES);
	global_dev = dev;

	fpi_ssm_start(global_enroll_ssm, enroll_done);

	return 0;
}

static int enroll_stop(struct fp_dev *dev)
{
	fpi_drvcb_enroll_stopped(global_dev);
	
	return 0;
}

static void verify_done(struct fpi_ssm *ssm)
{
	struct fp_dev *dev = ssm->dev;

	fpi_ssm_free(ssm);

	global_verify_ssm = NULL;
	global_enroll_ssm = NULL;
}


static int verify_start(struct fp_dev *dev)
{
	global_verify_ssm = fpi_ssm_new(dev, verify_start_sm_run_state,
			VERIFY_START_NUM_STATES);
	global_dev = dev;

	fpi_ssm_start(global_verify_ssm, verify_done);

	return 0;
}

static int verify_stop(struct fp_dev *dev, gboolean iterating)
{
	fpi_drvcb_verify_stopped(dev);
	return 0;
}

static void dev_exit(struct fp_dev *dev)
{
	fpi_drvcb_close_complete(dev);
	global_dev = NULL;
	global_enroll_ssm = NULL;
	global_verify_ssm = NULL;
}

static const struct usb_id id_table[] = {
	{ .vendor = 0x1c7a, .product = 0x603 }, /* Egistech */
	{ 0, 0, 0, },
};

struct fp_driver vkapi_driver = {
	.id = 1,
	.name = FP_COMPONENT,
	.full_name = "VKAPI",
	.id_table = id_table,
	.scan_type = FP_SCAN_TYPE_SWIPE,
	.open = dev_init,
	.close = dev_exit,
	.enroll_start = enroll_start,
	.enroll_stop = enroll_stop,
	.verify_start = verify_start,
	.verify_stop = verify_stop,
};
