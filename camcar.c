#include <stdlib.h>
#include <initio.h>
#include <curses.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <pthread.h>
#include <assert.h>
#include "detect_blob.h"

// Constants defining the minimum and maximum distance (in cm) for maintaining proper distance
#define DIST_MIN 60
#define DIST_MAX 100

// Structure used for communication between the main thread and the camera thread
struct thread_dat {
    TBlobSearch blob;  // Holds the blob object detected by the camera
    int blobnr;        // Tracks the blob number, indicating when a new image is produced
    int bExit;         // Flag used to signal thread termination
};

// Mutex for protecting shared data between threads
pthread_mutex_t count_mutex;

// Main function implementing the hierarchical finite state machines (FSMs) for car control
void camcar(int argc, char *argv[], struct thread_dat *ptdat) 
{
    int ch = 0;  // Variable to store user input key
    int blobnr = 0;  // Tracks the blob number processed in the last cycle
    TBlobSearch blob;  // Blob object for processing

    // Main control loop
    while (ch != 'q') {
        int obstacle_L, obstacle_R, obstacle;  // Variables for obstacle detection
        int blobSufficient;  // Indicates whether the detected blob is of sufficient size
        int carBlobAligned;  // Indicates whether the car is aligned with the blob
        int distance;  // Variable to store the distance to an object
        enum { distok, tooclose, toofar } distanceState;  // FSM states for maintaining distance

        // Display program instructions
        mvprintw(1, 1, "%s: Press 'q' to end program", argv[0]);

        // Acquire blob data from the camera thread with mutex protection
        pthread_mutex_lock(&count_mutex);
        blob = ptdat->blob;
        pthread_mutex_unlock(&count_mutex);

        // Display the current blob data
        mvprintw(10, 1, "Status: blob(size=%d, halign=%f, blobnr=%u)", blob.size, blob.halign, ptdat->blobnr);

        // Read obstacle sensors
        obstacle_L = (initio_IrLeft() != 0);
        obstacle_R = (initio_IrRight() != 0);
        obstacle = obstacle_L || obstacle_R;

        // FSM for obstacle avoidance
        if (obstacle) {
            mvprintw(3, 1, "State OA (stop to avoid obstacle), o-left=%d, o-right=%d", obstacle_L, obstacle_R);
            clrtoeol();  // Clear to end of line
            initio_DriveForward(0);  // Stop the car
        } else {
            refresh();  // Update display

            blobSufficient = (blob.size > 20);  // Check if the blob size is above the threshold

            // FSM for searching a blob
            if (!blobSufficient) {
                mvprintw(3, 1, "State SB (search blob), blob.size=%d (blobnr: %u)", blob.size, ptdat->blobnr);
                clrtoeol();
                if (blobnr < ptdat->blobnr) {
                    // Turn the car slightly to search for a blob
                    initio_SpinLeft(50);
                    delay(200);
                    initio_DriveForward(0);
                    blobnr = ptdat->blobnr;
                }
            } else {
                carBlobAligned = (blob.halign >= -0.25 && blob.halign <= 0.25);  // Check alignment with blob

                // FSM for aligning to a blob
                if (!carBlobAligned) {
                    mvprintw(3, 1, "State AB (align towards blob), blob.size=%d, halign=%f", blob.size, blob.halign);
                    clrtoeol();
                    if (blobnr < ptdat->blobnr) {
                        if (blob.halign < 0) {
                            initio_SpinRight(40);
                            delay(150);
                        } else {
                            initio_SpinLeft(40);
                            delay(150);
                        }
                        initio_DriveForward(0);
                        blobnr = ptdat->blobnr;
                    }
                } else {
                    distance = initio_UsGetDistance();  // Get distance from ultrasonic sensor
                    if (distance < DIST_MIN) {
                        distanceState = tooclose;
                    } else if (distance > DIST_MAX) {
                        distanceState = toofar;
                    } else {
                        distanceState = distok;
                    }

                    // FSM for maintaining proper blob distance
                    switch (distanceState) {
                        case toofar:
                            mvprintw(3, 1, "State FB (drive forward), dist=%d", distance);
                            clrtoeol();
                            initio_DriveForward(40);  // Move forward slowly
                            break;
                        case tooclose:
                            mvprintw(3, 1, "State RB (drive backwards), dist=%d", distance);
                            clrtoeol();
                            initio_DriveReverse(40);  // Move backward slowly
                            break;
                        case distok:
                            mvprintw(3, 1, "State KD (keep distance), dist=%d", distance);
                            clrtoeol();
                            initio_DriveForward(0);  // Maintain current position
                            break;
                    }
                }
            }
        }

        // Handle user input for quitting
        ch = getch();
        if (ch != ERR) mvprintw(2, 1, "Key code: '%c' (%d)", ch, ch);
        refresh();  // Update display
    }
}

// Thread function to continuously process camera images and detect blobs
void *worker(void *p_thread_dat) 
{
    struct thread_dat *ptdat = (struct thread_dat *) p_thread_dat;
    const char blobColor[3] = {255, 0, 0};  // Target blob color (red)
    TBlobSearch blob;

    while (ptdat->bExit == 0) {
        blob = cameraSearchBlob(blobColor);  // Detect red-colored blobs

        // Copy detected blob data to shared structure with mutex protection
        pthread_mutex_lock(&count_mutex);
        ptdat->blob = blob;
        ptdat->blobnr++;
        pthread_mutex_unlock(&count_mutex);
    }
    return NULL;
}

// Main function to initialize resources and start the threads
int main(int argc, char *argv[]) 
{
    WINDOW *mainwin = initscr();  // Initialize curses library
    noecho();
    cbreak();
    nodelay(mainwin, TRUE);
    keypad(mainwin, TRUE);

    initio_Init();  // Initialize robot control library
    pthread_mutex_init(&count_mutex, NULL);  // Initialize mutex

    pthread_t cam_thread;  // Thread handle for camera processing
    pthread_attr_t pt_attr;  // Thread attributes
    struct thread_dat tdat = {0};  // Shared data structure

    pthread_attr_init(&pt_attr);  // Initialize thread attributes
    pthread_create(&cam_thread, &pt_attr, worker, (void*)&tdat);  // Create worker thread

    camcar(argc, argv, &tdat);  // Start main control loop

    tdat.bExit = 1;  // Signal worker thread to exit
    pthread_join(cam_thread, NULL);  // Wait for worker thread to finish
    pthread_attr_destroy(&pt_attr);  // Destroy thread attributes

    pthread_mutex_destroy(&count_mutex);  // Destroy mutex
    initio_Cleanup();  // Cleanup robot resources
    endwin();  // Cleanup curses library
    return EXIT_SUCCESS;
}
