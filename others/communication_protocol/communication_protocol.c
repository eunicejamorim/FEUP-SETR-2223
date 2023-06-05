#define RECEIVER    6
#define TRANSMITTER 7
#define FLAG        0x7e
#define ESCAPE      0x7d

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

uint8_t calculate_bcc(uint8_t *data, uint8_t length) {
    uint8_t bcc = 0;
    for (int i = 0; i < length; ++i) {
        bcc = data[i] ^ bcc;
    }
    return bcc;
}

uint8_t stuffing(uint8_t **dst, uint8_t *src, uint8_t length) {
    // Calculating new length
    uint8_t new_length = length;
    for (int i = 0; i < length; ++i) {
        if (src[i] == FLAG || src[i] == ESCAPE) {
            new_length++;
        }
    }

    // Allocating new data
    *dst = (uint8_t*) malloc(new_length * sizeof(uint8_t));
    if (*dst == NULL) {
        printf("Error on dst malloc");
        return 0;
    }

    // Data stuffing
    int x = 0;
    for (int y = 0; y < length; ++y) {
        if (src[y] == FLAG || src[y] == ESCAPE) {
            (*dst)[x] = ESCAPE;
            (*dst)[x+1] = src[y] ^ 0x20;
            x += 2;
        } else {
            (*dst)[x] = src[y];
            x++;
        }
    }

    return new_length;
}

uint8_t destuffing(uint8_t **dst, uint8_t *src, uint8_t length) {
    // Calculating new length
    uint8_t new_length = length;
    for (int i = 0; i < length - 1; ++i) {
        if (src[i] == ESCAPE && (src[i+1] == FLAG ^ 0x20 || src[i+1] == ESCAPE ^ 0x20)) {
            new_length--;
        }
    }

    // Allocating new data
    *dst = (uint8_t*) malloc(new_length * sizeof(uint8_t));
    if (*dst == NULL) {
        printf("Error on dst malloc\n");
        return 0;
    }

    // Data stuffing
    int x = 0;
    for (int y = 0; y < new_length; ++y) {
        if (src[x] == ESCAPE && src[x+1] == (FLAG ^ 0x20)) {
            (*dst)[y] = FLAG;
            x += 2;
        } else if (src[x] == ESCAPE && src[x+1] == (ESCAPE ^ 0x20)) {
            (*dst)[y] = ESCAPE;
            x += 2;
        } else {
            (*dst)[y] = src[x];
            x++;
        }
    }

    return new_length;
}

uint8_t create_frame(uint8_t **dst, uint8_t *src, uint8_t length) {
    uint8_t frame_length = length + 3;
    uint8_t *frame_before_stuffing = (uint8_t *) malloc(frame_length * sizeof(uint8_t));
    memcpy(frame_before_stuffing + 1, src, length);
    frame_before_stuffing[0] = 0;
    frame_before_stuffing[frame_length - 2] = calculate_bcc(frame_before_stuffing + 1, length);
    frame_before_stuffing[frame_length - 1] = 0;
    if ((frame_length = stuffing(dst, frame_before_stuffing, frame_length)) == 0) {
        free(frame_before_stuffing);
        return 0;
    }
    (*dst)[0] = FLAG;
    (*dst)[frame_length - 1] = FLAG;
    return frame_length;
}

uint8_t get_frame_data(uint8_t **dst, uint8_t *src, uint8_t length) {
    uint8_t *frame = NULL;
    uint8_t frame_length = destuffing(&frame, src, length);
    uint8_t data_length = frame_length - 3;
    if (frame[frame_length - 2] != calculate_bcc(frame + 1, data_length)) {
        printf("Invalid BCC\n");
        return 0;
    }
    *dst = (uint8_t *) malloc(data_length * sizeof(uint8_t));
    if (*dst == NULL) {
        printf("Error on dst malloc\n");
        return 0;
    }
    memcpy(*dst, frame + 1, data_length);
    return data_length;
}

void main() {
    uint8_t data_length = 2;
    uint8_t *data = (uint8_t*) malloc(data_length * sizeof(uint8_t));
    data[0] = FLAG;
    data[1] = ESCAPE;

    printf("INITIAL_DATA:\n");
    for (int i = 0; i < data_length; ++i) {
        printf("%#010x\n", data[i]);
    }

    uint8_t *frame = NULL;
    uint8_t frame_length = create_frame(&frame, data, data_length);
    free(data);
    
    printf("FRAME:\n");
    for (int i = 0; i < frame_length; ++i) {
        printf("%#010x\n", frame[i]);
    }

    data_length = get_frame_data(&data, frame, frame_length);
    free(frame);

    printf("FINAL_DATA:\n");
    for (int i = 0; i < data_length; ++i) {
        printf("%#010x\n", data[i]);
    }

    free(data);
}
