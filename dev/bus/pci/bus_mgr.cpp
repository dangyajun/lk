/*
 * Copyright (c) 2021 Travis Geiseblrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */

#include "bus_mgr.h"

#include <sys/types.h>
#include <lk/cpp.h>
#include <lk/debug.h>
#include <lk/err.h>
#include <lk/list.h>
#include <lk/trace.h>
#include <dev/bus/pci.h>
#include <stdio.h>
#include <assert.h>

#define LOCAL_TRACE 2

namespace pci {

// root of the pci bus
class bus;
bus *root = nullptr;

// generic pci device
class device {
public:
    device(pci_location_t loc, bus *bus) : loc_(loc), bus_(bus) {}
    virtual ~device() = default;

    DISALLOW_COPY_ASSIGN_AND_MOVE(device);

    static status_t probe(pci_location_t loc, bus *bus, device **out_device, bool *out_multifunction);

    pci_location_t loc() const { return loc_; }
    const bus *get_bus() const { return bus_; }

    virtual void dump(size_t indent = 0);

protected:
    // let the bus device directly manipulate our list node
    friend class bus;
    list_node *list_node_ptr() { return &node; }
    list_node node = LIST_INITIAL_CLEARED_VALUE;

    pci_location_t loc_ = {};
    bus *bus_ = nullptr;

    pci_config_t config_ = {};
};

// bridge device, holds a list of busses that it is responsible for
class bridge : public device {
public:
    bridge(pci_location_t loc, bus *bus) : device(loc, bus) {}
    virtual ~bridge() = default;

    DISALLOW_COPY_ASSIGN_AND_MOVE(bridge);

    static status_t probe(pci_location_t loc, bus *bus, device **out_device);

    void add_bus(bus *b);

    virtual void dump(size_t indent = 0);

private:
    pci_bridge_config_t bridge_config_ = {};
    list_node child_busses_ = LIST_INITIAL_VALUE(child_busses_);
};

// bus device holds a list of devices and a reference to its bridge device
class bus {
public:
    explicit bus(pci_location_t loc, bridge *b) : loc_(loc), b_(b) {}
    virtual ~bus() = default;

    DISALLOW_COPY_ASSIGN_AND_MOVE(bus);

    static status_t probe(pci_location_t loc, bridge *bridge, bus **out_bus);

    pci_location_t loc() const { return loc_; }
    uint bus_num() const { return loc().bus; }

    void add_device(device *d);

    void dump(size_t indent = 0);

private:
    // let the bridge device directly manipulate our list node
    friend class bridge;
    list_node *list_node_ptr() { return &node; }
    list_node node = LIST_INITIAL_CLEARED_VALUE;

    pci_location_t loc_ = {};
    bridge *b_ = nullptr;
    list_node child_devices_ = LIST_INITIAL_VALUE(child_devices_);
};

void bus::add_device(device *d) {
    // TODO: assert that no two devices have the same address
    list_add_tail(&child_devices_, d->list_node_ptr());
}

void bridge::add_bus(bus *b) {
    list_add_tail(&child_busses_, b->list_node_ptr());
}

// helper routines
namespace {
const char *pci_loc_string(pci_location_t loc, char out_str[14]) {
    snprintf(out_str, 14, "%04x:%02x:%02x.%1x", loc.segment, loc.bus, loc.dev, loc.fn);
    return out_str;
}

status_t read_type0_config(pci_location_t loc, pci_config_t *config) {
    for (size_t i = 0; i < sizeof(pci_config_t); i++) {
        // TODO: handle endian swap
        status_t err = pci_read_config_byte(&loc, i, (uint8_t *)config + i);
        if (err != _PCI_SUCCESSFUL) {
            return ERR_NOT_FOUND;
        }
    }
    return NO_ERROR;
}

status_t read_type1_config(pci_location_t loc, pci_bridge_config_t *config) {
    for (size_t i = 0; i < sizeof(pci_bridge_config_t); i++) {
        // TODO: handle endian swap
        status_t err = pci_read_config_byte(&loc, i, (uint8_t *)config + i);
        if (err != _PCI_SUCCESSFUL) {
            return ERR_NOT_FOUND;
        }
    }
    return NO_ERROR;
}
} // anonymous namespace

// probe the device, return a new node and a bool if it's a multifunction device or not
status_t device::probe(pci_location_t loc, bus *parent_bus, device **out_device, bool *out_multifunction) {
    status_t err;

    *out_device = nullptr;
    *out_multifunction = false;

    // read vendor id and see if this is a real device
    uint16_t vendor_id;
    err = pci_read_config_half(&loc, PCI_CONFIG_VENDOR_ID, &vendor_id);
    if (err != NO_ERROR) {
        return ERR_NOT_FOUND;
    }
    if (vendor_id == 0xffff) {
        return ERR_NOT_FOUND;
    }

    char str[14];
    LTRACEF("something at %s\n", pci_loc_string(loc, str));

    // read base and sub class
    uint8_t base_class;
    err = pci_read_config_byte(&loc, PCI_CONFIG_CLASS_CODE_BASE, &base_class);
    if (err != NO_ERROR) {
        return ERR_NOT_FOUND;
    }
    uint8_t sub_class;
    err = pci_read_config_byte(&loc, PCI_CONFIG_CLASS_CODE_SUB, &sub_class);
    if (err != NO_ERROR) {
        return ERR_NOT_FOUND;
    }

    // read header type (0 or 1)
    uint8_t header_type;
    err = pci_read_config_byte(&loc, PCI_CONFIG_HEADER_TYPE, &header_type);
    if (err != NO_ERROR) {
        return ERR_NOT_FOUND;
    }

    // is it multifunction?
    bool possibly_multifunction = false;
    if (loc.fn == 0 && ~header_type & PCI_HEADER_TYPE_MULTI_FN) {
        possibly_multifunction = true;
    }

    header_type &= 0xf; // bottom 4 bits

    LTRACEF_LEVEL(2, "base:sub class %#hhx:%hhx\n", base_class, sub_class);

    // if it's a bridge, probe that
    if (base_class == 0x6) { // XXX replace with #define
        // bridge
        if (sub_class == 0x4) { // PCI-PCI bridge, normal decode
            LTRACEF("found bridge, recursing\n");
            return bridge::probe(loc, parent_bus, out_device);
        }
    }

    LTRACEF_LEVEL(2, "type %#hhx\n", header_type);

    if (header_type != 0) {
        LTRACEF("type %d header on bridge we don't understand, skipping\n", header_type);
        return ERR_NOT_FOUND;
    }

    // create a new device and pass it up
    device *d = new device(loc, parent_bus);

    // try to read in the basic config space for this device
    err = read_type0_config(loc, &d->config_);
    if (err < 0) {
        delete d;
        return err;
    }

    // we know we're a device at this point, set multifunction or not
    *out_multifunction = possibly_multifunction;

    // return the newly constructed device
    *out_device = d;

    return NO_ERROR;
}

void device::dump(size_t indent) {
    for (size_t i = 0; i < indent; i++) {
        printf(" ");
    }
    char str[14];
    printf("dev %s %04hx:%04hx\n", pci_loc_string(loc_, str), config_.vendor_id, config_.device_id);
}

// examine the bridge device, figuring out the bus range it controls and recurse
status_t bridge::probe(pci_location_t loc, bus *parent_bus, device **out_device) {
    char str[14];
    LTRACEF("%s\n", pci_loc_string(loc, str));

    // read vendor id and see if this is a real device
    uint16_t vendor_id;
    status_t err = pci_read_config_half(&loc, PCI_CONFIG_VENDOR_ID, &vendor_id);
    if (err != NO_ERROR) {
        return ERR_NOT_FOUND;
    }
    if (vendor_id == 0xffff) {
        return ERR_NOT_FOUND;
    }

    // read header type (0 or 1)
    uint8_t header_type;
    err = pci_read_config_byte(&loc, PCI_CONFIG_HEADER_TYPE, &header_type);
    if (err != NO_ERROR) {
        return ERR_NOT_FOUND;
    }

    // we are a bridge to a new set of busses
    bridge *br = new bridge(loc, parent_bus);

    // we only grok type 1 headers here
    err = read_type1_config(loc, &br->bridge_config_);
    if (err < 0) {
        delete br;
        return err;
    }

    LTRACEF("primary bus %hhd secondary %hhd subordinate %hhd\n",
            br->bridge_config_.primary_bus, br->bridge_config_.secondary_bus,
            br->bridge_config_.subordinate_bus);

    *out_device = br;

    // start a scan of the secondary bus downstream of this.
    // via bridge devices on this bus, should find all of the subordinate busses.
    bus *new_bus;
    pci_location_t bus_location = {};
    bus_location.segment = loc.segment;
    bus_location.bus = br->bridge_config_.secondary_bus;
    err = bus::probe(bus_location, br, &new_bus);
    if (err < 0) {
        return err;
    }

    // add the bus to our list of children
    DEBUG_ASSERT(new_bus);
    br->add_bus(new_bus);

    return NO_ERROR;
}

void bridge::dump(size_t indent) {
    for (size_t i = 0; i < indent; i++) {
        printf(" ");
    }
    char str[14];
    printf("bridge %s %04hx:%04hx child busses [%d..%d]\n", pci_loc_string(loc_, str),
           bridge_config_.vendor_id, bridge_config_.device_id,
           bridge_config_.secondary_bus, bridge_config_.subordinate_bus);

    bus *b;
    list_for_every_entry(&child_busses_, b, bus, node) {
        b->dump(indent + 2);
    }
}

// walk all devices on a bus recursively walking into any bridge and scanning those busses
status_t bus::probe(pci_location_t loc, bridge *bridge, bus **out_bus) {
    char str[14];
    LTRACEF("%s\n", pci_loc_string(loc, str));

    status_t err;

    // create a bus to hold any devices we find
    bus *b = new bus(loc, bridge);

    // probe all functions on all 32 devices on this bus.
    // add any devices found to this bus
    for (uint dev = 0; dev < 32; dev++) {
        loc.dev = dev;
        for (uint fn = 0; fn < 8; fn++) {
            loc.fn = fn;

            device *d;
            bool multifunction;
            err = device::probe(loc, b, &d, &multifunction);
            if (err < 0) {
                break;
            }

            b->add_device(d);

            // move on to the next device
            if (fn == 0 && !multifunction) {
                break;
            }
        }
    }

    *out_bus = b;

    return NO_ERROR;
}

void bus::dump(size_t indent) {
    for (size_t i = 0; i < indent; i++) {
        printf(" ");
    }
    printf("bus %d\n", loc().bus);
    device *d;
    list_for_every_entry(&child_devices_, d, device, node) {
        d->dump(indent + 2);
    }
}

status_t bus_mgr_init() {
    LTRACE_ENTRY;

    // start drilling into the pci bus tree
    pci_location_t loc;

    loc = {}; // start at 0:0:0.0

    bus *b;
    // TODO: deal with root bus not having reference to bridge device
    status_t err = bus::probe(loc, nullptr, &b);
    if (err < 0) {
        return err;
    }

    // if we found anything there should be at least an empty bus device
    DEBUG_ASSERT(b);
    root = b;

    // iterate over all the devices found
    printf("PCI dump:\n");
    root->dump(2);

    return NO_ERROR;
}

}
