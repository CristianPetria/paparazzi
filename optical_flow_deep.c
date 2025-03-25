#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include "video_thread_nps.h"
#include "paparazzi_uav.h"  // Assuming this exists for sending commands

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define JITTER_AMOUNT 2  // Pixels to jitter
#define FLOW_THRESHOLD 50  // Threshold for detecting high optical flow
#define AVOIDANCE_ANGLE 15  // Degrees to turn when avoiding

uint8_t prev_frame[FRAME_HEIGHT][FRAME_WIDTH];  // Previous frame storage

void compute_optical_flow(uint8_t current_frame[FRAME_HEIGHT][FRAME_WIDTH]) {
    int high_flow_count = 0;
    int flow_x = 0, flow_y = 0;

    for (int y = 1; y < FRAME_HEIGHT - 1; y++) {
        for (int x = 1; x < FRAME_WIDTH - 1; x++) {
            int flow = abs(current_frame[y][x] - prev_frame[y][x]);  // Compute pixel difference
            if (flow > FLOW_THRESHOLD) {
                high_flow_count++;
                flow_x += x;
                flow_y += y;
            }
        }
    }

    if (high_flow_count > 100) {  // Detected significant motion
        int avg_x = flow_x / high_flow_count;
        int avg_y = flow_y / high_flow_count;

        // Determine turn direction
        if (avg_x < FRAME_WIDTH / 2) {
            printf("Avoiding left\n");
            send_heading_change(-AVOIDANCE_ANGLE);  // Turn left
        } else {
            printf("Avoiding right\n");
            send_heading_change(AVOIDANCE_ANGLE);  // Turn right
        }
    }

    // Store current frame as previous frame
    memcpy(prev_frame, current_frame, sizeof(prev_frame));
}

void jitter_frame(uint8_t frame[FRAME_HEIGHT][FRAME_WIDTH]) {
    int shift_x = (rand() % (2 * JITTER_AMOUNT + 1)) - JITTER_AMOUNT;
    int shift_y = (rand() % (2 * JITTER_AMOUNT + 1)) - JITTER_AMOUNT;

    uint8_t temp_frame[FRAME_HEIGHT][FRAME_WIDTH];
    memset(temp_frame, 0, sizeof(temp_frame));

    for (int y = JITTER_AMOUNT; y < FRAME_HEIGHT - JITTER_AMOUNT; y++) {
        for (int x = JITTER_AMOUNT; x < FRAME_WIDTH - JITTER_AMOUNT; x++) {
            int new_x = x + shift_x;
            int new_y = y + shift_y;
            if (new_x >= 0 && new_x < FRAME_WIDTH && new_y >= 0 && new_y < FRAME_HEIGHT) {
                temp_frame[new_y][new_x] = frame[y][x];
            }
        }
    }
    memcpy(frame, temp_frame, sizeof(temp_frame));
}

void video_thread_function(void *ptr) {
    while (1) {
        uint8_t frame[FRAME_HEIGHT][FRAME_WIDTH];

        // Grab frame
        video_grab_frame(frame);

        // Jitter the frame
        jitter_frame(frame);

        // Compute optical flow
        compute_optical_flow(frame);

        // Send frame for visualization
        viewvideo_send_frame(frame);

        usleep(1000);  // Wait 1 ms before processing the next frame
    }
}
can we try dis\