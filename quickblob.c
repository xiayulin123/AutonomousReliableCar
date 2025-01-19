#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "quickblob.h"

// Structure for managing a list of blobs during image processing
struct blob_list {
    struct blob* head;           // Head of the blob list
    int length;                  // Number of blobs allocated
    struct blob** empties;       // Stack of unused blob pointers
    int empty_i;                 // Index of the top of the empty stack
};

// Resets a blob structure to its initial state
static void blank(struct blob* b) {
    b->size = 0;
    b->color = -1;
    b->x1 = -1;
    b->x2 = -1;
    b->y = -1;
    b->prev = NULL;
    b->next = NULL;
    b->sib_p = NULL;
    b->sib_n = NULL;
    b->center_x = 0.0;
    b->center_y = 0.0;
    b->bb_x1 = b->bb_y1 = b->bb_x2 = b->bb_y2 = -1;
}

// Initializes a stream for reading pixel data
static int init_pixel_stream(void* user_struct, struct stream_state* stream) {
    memset(stream, 0, sizeof(struct stream_state));
    if (init_pixel_stream_hook(user_struct, stream)) {
        return 1;
    }
    stream->row = (unsigned char*) malloc(stream->w * sizeof(unsigned char));
    if (!stream->row) {
        return 1; // Memory allocation failure
    }
    stream->x = 0;
    stream->y = -1;
    stream->wrap = 0;
    return 0;
}

// Cleans up resources used by a pixel stream
static int close_pixel_stream(void* user_struct, struct stream_state* stream) {
    close_pixel_stream_hook(user_struct, stream);
    free(stream->row);
    stream->row = NULL;
    return 0;
}

// Allocates memory for blobs in the blob list
static int malloc_blobs(struct blob_list* blist) {
    blist->head = (struct blob*) malloc(blist->length * sizeof(struct blob));
    if (!blist->head) {
        return 1;
    }
    blist->empties = (struct blob**) malloc(blist->length * sizeof(struct blob*));
    if (!blist->empties) {
        return 1;
    }
    return 0;
}

// Initializes the blob list by resetting all blobs and linking unused blobs
static int init_blobs(struct blob_list* blist) {
    int i;
    struct blob* head = blist->head;
    blist->empty_i = 0;
    for (i = 0; i < blist->length; i++) {
        blank(&(head[i]));
        if (i > 0) {
            blist->empties[blist->empty_i++] = &(head[i]);
        }
    }
    return 0;
}

// Removes a blob from the linked list
static void blob_unlink(struct blob* b2) {
    struct blob* b1 = b2->prev;
    struct blob* b3 = b2->next;
    if (b1) b1->next = b3;
    if (b3) b3->prev = b1;
    b2->prev = b2->next = NULL;

    struct blob* s1 = b2->sib_p;
    struct blob* s3 = b2->sib_n;
    if (s1) s1->sib_n = s3;
    if (s3) s3->sib_p = s1;
    b2->sib_p = b2->sib_n = NULL;
}

// Inserts a blob into the sorted list
static void blob_insert(struct blob* bl_start, struct blob* b2) {
    struct blob* b1 = bl_start->prev;
    struct blob* b3;
    while (b1->next) {
        b3 = b1->next;
        if (b1->x1 <= b2->x1 && b2->x1 <= b3->x1) {
            b1->next = b2;
            b2->prev = b1;
            b2->next = b3;
            b3->prev = b2;
            return;
        }
        b1 = b1->next;
    }
    b1->next = b2;
    b2->prev = b1;
}

// Reads the next row of pixel data in the stream
static int next_row(void* user_struct, struct stream_state* stream) {
    if (stream->y >= stream->h) {
        return 1; // End of the stream
    }
    stream->wrap = 0;
    stream->x = 0;
    stream->y++;
    return next_row_hook(user_struct, stream);
}

// Reads the next frame in the stream
static int next_frame(void* user_struct, struct stream_state* stream) {
    stream->wrap = 0;
    stream->x = 0;
    stream->y = -1;
    return next_frame_hook(user_struct, stream);
}

// Scans a segment of pixels in the current row
static int scan_segment(struct stream_state* stream, struct blob* b) {
    if (stream->wrap) return 1; // End of row
    b->x1 = stream->x;
    b->color = stream->row[stream->x];
    while (stream->x < stream->w && b->color == stream->row[stream->x]) {
        stream->x++;
    }
    b->x2 = stream->x - 1;
    return 0;
}

// Updates the bounding box of a blob
static void bbox_update(struct blob* b, int x1, int x2, int y1, int y2) {
    if (b->bb_x1 < 0 || x1 < b->bb_x1) b->bb_x1 = x1;
    if (x2 > b->bb_x2) b->bb_x2 = x2;
    if (b->bb_y1 < 0 || y1 < b->bb_y1) b->bb_y1 = y1;
    if (y2 > b->bb_y2) b->bb_y2 = y2;
}

// Updates the properties of a blob with new pixel information
static void blob_update(struct blob* b, int x1, int x2, int y) {
    int s2 = 1 + x2 - x1;
    b->center_x = ((b->center_x * b->size) + (x1 + x2) * s2 / 2) / (b->size + s2);
    b->center_y = ((b->center_y * b->size) + (y * s2)) / (b->size + s2);
    b->size += s2;
    bbox_update(b, x1, x2, y, y);
}

// Links sibling blobs into a single list
static void sib_link(struct blob* b1, struct blob* b2) {
    while (b1->sib_p) b1 = b1->sib_p;
    while (b2->sib_p) b2 = b2->sib_p;
    if (b1 == b2) return; // Already linked

    struct blob* tmp;
    while (b1 && b2) {
        if (b2->x1 < b1->x1) {
            tmp = b1;
            b1 = b2;
            b2 = tmp;
            continue;
        }
        if (b1->sib_n && b1->sib_n->x1 < b2->x1) {
            b1 = b1->sib_n;
            continue;
        }
        tmp = b1->sib_n;
        b1->sib_n = b2;
        b2->sib_p = b1;
        b1 = b2;
        b2 = tmp;
    }
}

// Extracts blobs from an image stream
int extract_image(void* user_struct) {
    struct stream_state stream;
    struct blob_list blist;
    struct blob* blob_now = NULL;
    struct blob* blob_prev = NULL;

    if (init_pixel_stream(user_struct, &stream)) {
        printf("Error initializing pixel stream.\n");
        return 1;
    }
    blist.length = stream.w + 5;
    if (malloc_blobs(&blist)) {
        printf("Error allocating blob list.\n");
        return 1;
    }

    while (!next_frame(user_struct, &stream)) {
        init_blobs(&blist);
        while (!next_row(user_struct, &stream)) {
            blob_prev = blist.head->next;
            while (!stream.wrap) {
                blob_now = empty_blob(&blist);
                if (scan_segment(&stream, blob_now)) {
                    blob_reap(&blist, blob_now);
                    continue;
                }
                blob_update(blob_now, blob_now->x1, blob_now->x2, stream.y);
                sib_find(blist.head->next, blob_now);
                blob_insert(blob_prev, blob_now);
                blob_prev = blob_now;
            }
            flush_old_blobs(user_struct, &blist, stream.y);
        }
    }

    close_pixel_stream(user_struct, &stream);
    free(blist.head);
    free(blist.empties);
    return 0;
}
