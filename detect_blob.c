#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <jpeglib.h>
#include <jerror.h>
#include <assert.h>
#include <string.h>
#include "detect_blob.h"
#include "quickblob.h"

// Macros for calculating the maximum and minimum of two values.
#define max(a,b)  ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define min(a,b)  ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

// Structure for managing image and blob search operations.
typedef struct QuickBlob {
  TJImage *pimg;          // Pointer to image data.
  char ref[3];            // RGB reference values for blob filtering.
  int frame;              // Frame counter (not used in single-image applications).
  double ref_rel[3];      // Normalized reference values relative to red component.
  struct blob blob_max;   // Largest detected blob.
} TQuickBlob;

// Macro to check if a pixel matches a reference color within a range.
#define BLOB_MATCH(ref,dat) (((ref)*0.9 <= (dat)) && ((dat) <= min(255,(ref)*1.1)))

// Command to capture an image using the Raspberry Pi camera.
#define CAMERA_CMD "raspistill -w 200 -h 200 -t 1 -awb fluorescent --nopreview --mode 7 -rot 270"

// Function prototypes for local hook functions used with QuickBlob.
void log_blob_hook(void* user_struct, struct blob* b);
int init_pixel_stream_hook(void* user_struct, struct stream_state* stream);
int close_pixel_stream_hook(void* user_struct, struct stream_state* stream);
int next_row_hook(void* user_struct, struct stream_state* stream);
int next_frame_hook(void* user_struct, struct stream_state* stream);

// Helper function to merge multiple strings into one dynamically allocated string.
static char* MergeStrings(int num_args, char* str1, ...);

// Helper function to print an error message and terminate the program.
void bailout(char *msg);

// Function to capture an image and search for the largest blob matching a specific color.
TBlobSearch cameraSearchBlob(const char color[3]) {
    TJImage img;
    img = capturePhoto();
    return imageSearchBlob(color, &img);
}

// Function to search an image for the largest blob of a specific color.
TBlobSearch imageSearchBlob(const char color[3], TJImage *pimg) {
    TBlobSearch blob_res;  // Structure to store the search result.
    TQuickBlob dblob;      // Structure for interfacing with QuickBlob.

    dblob.pimg = pimg;
    dblob.ref[0] = color[0];
    dblob.ref[1] = color[1];
    dblob.ref[2] = color[2];

    extract_image((void*)&dblob); // Search blobs in the image using QuickBlob.

    blob_res.blob = dblob.blob_max;
    blob_res.size = dblob.blob_max.size;

    if (dblob.blob_max.size > 0) {
        // Calculate alignment of the blob relative to the center of the image.
        blob_res.halign = -1.0 + 2.0 * ((double)(dblob.blob_max.center_x) / (dblob.pimg)->w);
        blob_res.valign = -1.0 + 2.0 * ((double)(dblob.blob_max.center_y) / (dblob.pimg)->h);
    }

    blob_res.pimg = pimg;
    return blob_res;
}

// Function to read JPEG image data using libjpeg.
TJImage read_JPEG_image(FILE *file) {
    struct jpeg_decompress_struct info; // JPEG decompression structure.
    struct jpeg_error_mgr err;          // Error handler for JPEG library.
    static unsigned char *img_data = NULL; // Buffer to hold image data.

    TJImage img;

    info.err = jpeg_std_error(&err);
    jpeg_create_decompress(&info);
    jpeg_stdio_src(&info, file);
    jpeg_read_header(&info, TRUE);
    jpeg_start_decompress(&info);

    img.w = info.output_width;
    img.h = info.output_height;
    img.numChannels = info.num_components; // Number of color channels (e.g., RGB or RGBA).

    unsigned long dataSize = img.w * img.h * img.numChannels;

    // Allocate or resize the image data buffer.
    if (img_data == NULL) {
        img_data = (unsigned char *)malloc(dataSize);
    } else {
        img_data = (unsigned char *)realloc(img_data, dataSize);
    }

    img.data = img_data;

    // Read each scanline into the image buffer.
    unsigned char* rowptr;
    while (info.output_scanline < img.h) {
        rowptr = img.data + info.output_scanline * img.w * img.numChannels;
        jpeg_read_scanlines(&info, &rowptr, 1);
    }

    jpeg_finish_decompress(&info);
    return img;
}

// Function to read a JPEG image from a file.
TJImage readJpegImageFromFile(const char *fname) {
    FILE *file;
    TJImage img;

    file = fopen(fname, "rb");
    if (file == NULL) {
        exit(EXIT_FAILURE);
    }

    img = read_JPEG_image(file);
    fclose(file);

    return img;
}

// Function to capture a photo using the Raspberry Pi camera and save it to a file.
int capturePhotoToFile(const char *fname) {
    FILE *fp;
    char *cmd;

    cmd = MergeStrings(4, CAMERA_CMD, " -o ", fname, " ");
    fp = popen(cmd, "r");

    if (fp == NULL) bailout("capturePhotoToFile() failed!");

    pclose(fp);
    free(cmd); // Free the dynamically allocated command string.
    return 0;
}

// Function to capture a photo using the Raspberry Pi camera and return raw image data.
TJImage capturePhoto() {
    FILE *fp;
    TJImage img;
    char *cmd;

    cmd = MergeStrings(2, CAMERA_CMD, " -o - ");
    fp = popen(cmd, "r");

    if (fp == NULL) bailout("capturePhoto() failed!");

    img = read_JPEG_image(fp);

    pclose(fp);
    free(cmd); // Free the dynamically allocated command string.
    return img;
}

// Function to save an image as a JPEG file with a specified quality.
void writeImageAsJPEG(TJImage *pimg, const char *fname, int quality) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE *outfile;
    JSAMPROW row_pointer; // Pointer to a single row of image data.
    int row_stride;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    outfile = fopen(fname, "wb");
    if (outfile == NULL) bailout("writeImageAsJPEG: error opening file");

    jpeg_stdio_dest(&cinfo, outfile);
    cinfo.image_width = pimg->w;
    cinfo.image_height = pimg->h;
    cinfo.input_components = pimg->numChannels;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    row_stride = pimg->w * pimg->numChannels;

    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer = &pimg->data[cinfo.next_scanline * row_stride];
        jpeg_write_scanlines(&cinfo, &row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    fclose(outfile);
}
