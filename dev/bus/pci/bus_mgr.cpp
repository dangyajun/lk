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

#define LOCAL_TRACE 0

namespace pci {

// root of the pci bus
class bus;
bus *root = nullptr;
list_node bus_list = LIST_INITIAL_VALUE(bus_list);

// generic pci device
class device {
public:
    device(pci_location_t loc, bus *bus) : loc_(loc), bus_(bus) {}
    virtual ~device() = default;

    DISALLOW_COPY_ASSIGN_AND_MOVE(device);

    static status_t probe(pci_location_t loc, bus *bus, device **out_device, bool *out_multifunction);

    pci_location_t loc() const { return loc_; }
    const bus *get_bus() const { return bus_; }

    uint16_t device_id() const { return config_.device_id; }
    uint16_t vendor_id() const { return config_.vendor_id; }
    uint8_t base_class() const { return config_.base_class; }
    uint8_t sub_class() const { return config_.sub_class; }
    uint8_t interface() const { return config_.program_interface; }

    virtual void dump(size_t indent = 0);

protected:
    // let the bus device directly manipulate our list node
    friend class bus;
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

    void add_bus(bus *b) { secondary_bus_ = b; }

    virtual void dump(size_t indent = 0);

private:
    bus *secondary_bus_ = nullptr;
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

    list_node *list_node_ptr() { return &node; }

    template <typename F>
    status_t for_every_device(F func);

    // master list of busses for easy iteration
    list_node node = LIST_INITIAL_CLEARED_VALUE;

private:
    pci_location_t loc_ = {};
    bridge *b_ = nullptr;
    list_node child_devices_ = LIST_INITIAL_VALUE(child_devices_);
};

void bus::add_device(device *d) {
    // TODO: assert that no two devices have the same address
    list_add_tail(&child_devices_, &d->node);
}

// helper routines
namespace {
const char *pci_loc_string(pci_location_t loc, char out_str[14]) {
    snprintf(out_str, 14, "%04x:%02x:%02x.%1x", loc.segment, loc.bus, loc.dev, loc.fn);
    return out_str;
}

// iterate all devices on all busses with the functor
template <typename F>
status_t for_every_device_on_every_bus(F func) {
    status_t err = NO_ERROR;

    bus *b;
    list_for_every_entry(&bus_list, b, bus, node) {
        err = b->for_every_device(func);
        if (err != NO_ERROR) {
            return err;
        }
    }
    return err;
}

// return a pointer to the device that matches a particular location
device *lookup_device_by_loc(pci_location_t loc) {
    device *ret = nullptr;

    auto v = [&](device *d) -> status_t {
        if (d->loc() == loc) {
            ret = d;
            return 1;
        }
    };

    for_every_device_on_every_bus(v);

    return ret;
}

} // namespace

// probe the device, return a new node and a bool if it's a multifunction device or not
status_t device::probe(pci_location_t loc, bus *parent_bus, device **out_device, bool *out_multifunction) {
    status_t err;

    *out_device = nullptr;
    *out_multifunction = false;

    // read vendor id and see if this is a real device
    uint16_t vendor_id;
    err = pci_read_config_half(loc, PCI_CONFIG_VENDOR_ID, &vendor_id);
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
    err = pci_read_config_byte(loc, PCI_CONFIG_CLASS_CODE_BASE, &base_class);
    if (err != NO_ERROR) {
        return ERR_NOT_FOUND;
    }
    uint8_t sub_class;
    err = pci_read_config_byte(loc, PCI_CONFIG_CLASS_CODE_SUB, &sub_class);
    if (err != NO_ERROR) {
        return ERR_NOT_FOUND;
    }

    // read header type (0 or 1)
    uint8_t header_type;
    err = pci_read_config_byte(loc, PCI_CONFIG_HEADER_TYPE, &header_type);
    if (err != NO_ERROR) {
        return ERR_NOT_FOUND;
    }

    // is it multifunction?
    bool possibly_multifunction = false;
    if (loc.fn == 0 && ~header_type & PCI_HEADER_TYPE_MULTI_FN) {
        possibly_multifunction = true;
    }

    header_type &= PCI_HEADER_TYPE_MASK;

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
    err = pci_read_config(loc, &d->config_);
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
    status_t err = pci_read_config_half(loc, PCI_CONFIG_VENDOR_ID, &vendor_id);
    if (err != NO_ERROR) {
        return ERR_NOT_FOUND;
    }
    if (vendor_id == 0xffff) {
        return ERR_NOT_FOUND;
    }

    // read header type (0 or 1)
    uint8_t header_type;
    err = pci_read_config_byte(loc, PCI_CONFIG_HEADER_TYPE, &header_type);
    if (err != NO_ERROR) {
        return ERR_NOT_FOUND;
    }

    // we are a bridge to a new set of busses
    bridge *br = new bridge(loc, parent_bus);

    // we only grok type 1 headers here
    err = pci_read_config(loc, &br->config_);
    if (err < 0) {
        delete br;
        return err;
    }

    LTRACEF("primary bus %hhd secondary %hhd subordinate %hhd\n",
            br->config_.type1.primary_bus, br->config_.type1.secondary_bus,
            br->config_.type1.subordinate_bus);

    *out_device = br;

    // start a scan of the secondary bus downstream of this.
    // via bridge devices on this bus, should find all of the subordinate busses.
    bus *new_bus;
    pci_location_t bus_location = {};
    bus_location.segment = loc.segment;
    bus_location.bus = br->config_.type1.secondary_bus;
    err = bus::probe(bus_location, br, &new_bus);
    if (err < 0) {
        return err;
    }

    // add the bus to our list of children
    DEBUG_ASSERT(new_bus);
    br->add_bus(new_bus);

    // add the bus to the global bus list
    list_add_tail(&bus_list, new_bus->list_node_ptr());

    return NO_ERROR;
}

void bridge::dump(size_t indent) {
    for (size_t i = 0; i < indent; i++) {
        printf(" ");
    }
    char str[14];
    printf("bridge %s %04hx:%04hx child busses [%d..%d]\n", pci_loc_string(loc_, str),
           config_.vendor_id, config_.device_id,
           config_.type1.secondary_bus, config_.type1.subordinate_bus);

    if (secondary_bus_) {
        secondary_bus_->dump(indent + 2);
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

// call the provided functor on every device in this bus
template <typename F>
status_t bus::for_every_device(F func) {
    status_t err = NO_ERROR;

    device *d;
    list_for_every_entry(&child_devices_, d, device, node) {
        err = func(d);
        if (err != NO_ERROR) {
            return err;
        }
    }
    return err;
}
} // namespace pci

// C api, so outside of the namespace
using namespace pci;

status_t pci_bus_mgr_init() {
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
    list_add_tail(&bus_list, b->list_node_ptr());

    // iterate over all the devices found
    printf("PCI dump:\n");
    root->dump(2);

    printf("visit all devices\n");
    pci_bus_mgr_visit_devices([](pci_location_t _loc) {
        char str[14];
        printf("%s\n", pci_loc_string(_loc, str));
    });

    return NO_ERROR;
}

// for every bus in the system, pass the visit routine to the device
status_t pci_bus_mgr_visit_devices(pci_visit_routine routine, void *cookie) {
    auto v = [&](device *d) -> status_t {
        routine(d->loc(), cookie);
        return NO_ERROR;
    };

    return for_every_device_on_every_bus(v);
}

status_t pci_bus_mgr_find_device(pci_location_t *state, uint16_t device_id, uint16_t vendor_id, size_t index) {
    LTRACEF("device_id dev %#hx vendor %#hx index %zu\n", device_id, vendor_id, index);

    if (device_id == 0xffff && vendor_id == 0xffff) {
        return ERR_INVALID_ARGS;
    }

    auto v = [&](device *d) -> status_t {

        if (device_id != 0xffff && device_id != d->device_id())
            return NO_ERROR;
        if (vendor_id != 0xffff && vendor_id != d->vendor_id())
            return NO_ERROR;

        if (index-- == 0) {
            char str[14];
            LTRACEF_LEVEL(2, "match at loc %s: device id %#hx vendor id %#hx\n", pci_loc_string(d->loc(), str), d->device_id(), d->vendor_id());
            *state = d->loc();
            return 1; // signals stop
        }
        return NO_ERROR;
    };

    status_t err = for_every_device_on_every_bus(v);
    return (err > 0) ? NO_ERROR : ERR_NOT_FOUND;
}

status_t pci_bus_mgr_find_device_by_class(pci_location_t *state, uint8_t base_class, uint8_t sub_class, uint8_t interface, size_t index) {
    LTRACEF("class %#x sub %#x interface %#x index %zu\n", base_class, sub_class, interface, index);

    if (sub_class == 0xff && interface == 0xff) {
        return ERR_INVALID_ARGS;
    }

    auto v = [&](device *d) -> status_t {
        //LTRACEF_LEVEL(2, "class %#x\n", d->base_class());

        if (base_class != d->base_class())
            return NO_ERROR;
        if (sub_class != 0xff && sub_class != d->sub_class())
            return NO_ERROR;
        if (interface != 0xff && interface != d->interface())
            return NO_ERROR;

        if (index-- == 0) {
            char str[14];
            LTRACEF_LEVEL(2, "match at loc %s: class %#hhx sub %#hhx interface %hhx\n",
                    pci_loc_string(d->loc(), str), d->base_class(), d->sub_class(), d->interface());
            *state = d->loc();
            return 1; // signals stop
        }
        return NO_ERROR;
    };

    status_t err = for_every_device_on_every_bus(v);
    return (err > 0) ? NO_ERROR : ERR_NOT_FOUND;
}

