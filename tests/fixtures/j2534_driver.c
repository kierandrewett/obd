/* Test-only J2534 04.04 DLL substitute. No vehicle or USB access. */
#include <stdint.h>
#include <string.h>

typedef struct {
    uint32_t protocol, rx_status, tx_flags, timestamp, data_size, extra_data_index;
    uint8_t data[4128];
} Message;

static uint32_t flags, filters, stage, request_size, scenario, closes, disconnects;
static uint8_t request[4095];

void MockScenario(uint32_t value) { scenario = value; }
uint32_t MockCloses(void) { return closes; }
uint32_t MockDisconnects(void) { return disconnects; }

uint32_t PassThruOpen(const void *name, uint32_t *device) {
    (void)name;
    *device = 11;
    filters = 0;
    return 0;
}
uint32_t PassThruClose(uint32_t device) {
    if (device != 11) return 0x1a;
    closes++;
    return 0;
}
uint32_t PassThruConnect(uint32_t device, uint32_t protocol, uint32_t connect_flags,
                         uint32_t baud, uint32_t *channel) {
    if (device != 11 || protocol != 6 || (baud != 250000 && baud != 500000)) return 3;
    flags = connect_flags;
    *channel = 22;
    return 0;
}
uint32_t PassThruDisconnect(uint32_t channel) {
    if (channel != 22) return 2;
    disconnects++;
    return 0;
}

static uint32_t id(const Message *message) {
    return ((uint32_t)message->data[0] << 24) | ((uint32_t)message->data[1] << 16)
        | ((uint32_t)message->data[2] << 8) | message->data[3];
}
uint32_t PassThruStartMsgFilter(uint32_t channel, uint32_t type, Message *mask,
                               Message *pattern, Message *flow, uint32_t *filter) {
    if (scenario == 1) return 0x12;
    if (channel != 22 || type != 3 || mask->data_size != 4 || pattern->data_size != 4) return 3;
    uint32_t response = flags ? 0x18DAF110 : 0x7e8 + filters;
    uint32_t flow_id = flags ? 0x18DA10F1 : 0x7e0 + filters;
    if (id(pattern) != response || id(flow) != flow_id) return 3;
    if (id(mask) != (flags ? 0x1fffffff : 0x7ff)) return 3;
    *filter = ++filters;
    return 0;
}
uint32_t PassThruIoctl(uint32_t handle, uint32_t command, void *input, void *output) {
    (void)input;
    if (command == 8 && handle == 22) { stage = 99; return 0; }
    if (command == 3 && handle == 11) { *(uint32_t *)output = 12600; return 0; }
    return 1;
}
uint32_t PassThruWriteMsgs(uint32_t channel, Message *message, uint32_t *count, uint32_t timeout) {
    (void)timeout;
    if (channel != 22 || *count != 1 || filters != (flags ? 1 : 8) || message->data_size > 4099) return 3;
    if (id(message) != (flags ? 0x18DB33F1 : 0x7df)) return 3;
    if (message->tx_flags != (flags | 0x40)) return 3;
    request_size = message->data_size - 4;
    memcpy(request, message->data + 4, request_size);
    stage = 0;
    return 0;
}
uint32_t PassThruReadMsgs(uint32_t channel, Message *message, uint32_t *count, uint32_t timeout) {
    (void)timeout;
    if (channel != 22 || *count != 1) return 3;
    if (stage > 3 || scenario == 2) { *count = 0; return 0x10; }
    memset(message, 0, sizeof(*message));
    message->protocol = 6;
    uint32_t address = flags ? 0x18DAF110 : 0x7e8;
    message->data[0] = address >> 24;
    message->data[1] = address >> 16;
    message->data[2] = address >> 8;
    message->data[3] = address;
    if (stage < 2) {
        message->rx_status = stage == 0 ? 8 : 2;
        message->data_size = 4;
    } else if (stage == 3 && scenario != 5) {
        *count = 0;
        stage++;
        return 9;
    } else if ((scenario == 5 || scenario == 6) && stage == 2) {
        uint8_t pending[] = {0x7f, request[0], 0x78};
        memcpy(message->data + 4, pending, 3);
        message->data_size = 7;
    } else if (scenario == 3) {
        message->data_size = 9999;
    } else if (scenario == 4) {
        uint8_t negative[] = {0x7f, request[0], 0x11};
        memcpy(message->data + 4, negative, 3);
        message->data_size = 7;
    } else {
        uint8_t *data = message->data + 4;
        if (request[0] == 1 && request_size == 2) {
            uint8_t reply[] = {0x41, request[1], 0xbe, 0x3f, 0xa8, 0x13};
            memcpy(data, reply, 6);
            message->data_size = 10;
        } else if (request[0] == 9) {
            data[0] = 0x49; data[1] = 2; data[2] = 1;
            memcpy(data + 3, "WF0A1234567890123", 17);
            message->data_size = 24;
        } else if (request[0] == 3) {
            uint8_t reply[] = {0x43, 2, 0x01, 0x33, 0xc1, 0x00};
            memcpy(data, reply, 6);
            message->data_size = 10;
        } else if (request[0] == 7) {
            data[0] = 0x47; data[1] = 0;
            message->data_size = 6;
        } else {
            data[0] = request[0] + 0x40;
            message->data_size = 5;
        }
    }
    *count = 1;
    stage++;
    return 0;
}
uint32_t PassThruGetLastError(char *buffer) {
    strcpy(buffer, "simulated driver error");
    return 0;
}
