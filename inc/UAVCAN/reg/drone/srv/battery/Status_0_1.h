/*
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020 NXP 
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// This is an AUTO-GENERATED UAVCAN DSDL data type implementation. Curious? See https://uavcan.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended unless it is used as part of a high-SIL
// safety-critical codebase. The typical usage scenario is to generate it as part of the build process.
//
// To avoid conflicts with definitions given in the source DSDL file, all entities created by the code generator
// are named with an underscore at the end, like foo_bar_().
//
// Generator:     nunavut-0.5.1 (serialization was enabled)
// Source file:   /home/hovergames/nuttx/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/srv/battery/Status.0.1.uavcan
// Generated at:  2020-11-18 09:49:35.456481 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     reg.drone.srv.battery.Status
// Version:       0.1

#ifndef REG_DRONE_SRV_BATTERY_STATUS_0_1_INCLUDED_
#define REG_DRONE_SRV_BATTERY_STATUS_0_1_INCLUDED_

#include <nunavut/support/serialization.h>
#include <reg/drone/srv/battery/Error_0_1.h>
#include <reg/drone/srv/common/Heartbeat_0_1.h>
#include <uavcan/si/unit/electric_charge/Scalar_1_0.h>
#include <uavcan/si/unit/temperature/Scalar_1_0.h>
#include <uavcan/si/unit/voltage/Scalar_1_0.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define reg_drone_srv_battery_Status_0_1_HAS_FIXED_PORT_ID_ false

#define reg_drone_srv_battery_Status_0_1_FULL_NAME_             "reg.drone.srv.battery.Status"
#define reg_drone_srv_battery_Status_0_1_FULL_NAME_AND_VERSION_ "reg.drone.srv.battery.Status.0.1"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define reg_drone_srv_battery_Status_0_1_EXTENT_BYTES_                    63UL
#define reg_drone_srv_battery_Status_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ 23UL
static_assert(reg_drone_srv_battery_Status_0_1_EXTENT_BYTES_ >= reg_drone_srv_battery_Status_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// Array metadata for: uavcan.si.unit.temperature.Scalar.1.0[2] temperature_min_max
#define reg_drone_srv_battery_Status_0_1_temperature_min_max_ARRAY_CAPACITY_           2U
#define reg_drone_srv_battery_Status_0_1_temperature_min_max_ARRAY_IS_VARIABLE_LENGTH_ false
/// Array metadata for: uavcan.si.unit.voltage.Scalar.1.0[2] cell_voltage_min_max
#define reg_drone_srv_battery_Status_0_1_cell_voltage_min_max_ARRAY_CAPACITY_           2U
#define reg_drone_srv_battery_Status_0_1_cell_voltage_min_max_ARRAY_IS_VARIABLE_LENGTH_ false

typedef struct
{
    /// reg.drone.srv.common.Heartbeat.0.1 heartbeat
    reg_drone_srv_common_Heartbeat_0_1 heartbeat;

    /// uavcan.si.unit.temperature.Scalar.1.0[2] temperature_min_max
    uavcan_si_unit_temperature_Scalar_1_0 temperature_min_max[2];

    /// uavcan.si.unit.voltage.Scalar.1.0[2] cell_voltage_min_max
    uavcan_si_unit_voltage_Scalar_1_0 cell_voltage_min_max[2];

    /// uavcan.si.unit.electric_charge.Scalar.1.0 available_charge
    uavcan_si_unit_electric_charge_Scalar_1_0 available_charge;

    /// reg.drone.srv.battery.Error.0.1 error
    reg_drone_srv_battery_Error_0_1 _error;
} reg_drone_srv_battery_Status_0_1;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see reg_drone_srv_battery_Status_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_drone_srv_battery_Status_0_1_serialize_(
    const reg_drone_srv_battery_Status_0_1* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }

    const size_t capacity_bytes = *inout_buffer_size_bytes;
    if ((8U * (size_t) capacity_bytes) < 184UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;

    {   // reg.drone.srv.common.Heartbeat.0.1 heartbeat
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 16ULL) <= (capacity_bytes * 8U));
        size_t _size_bytes0_ = 2UL;  // Nested object (max) size, in bytes.
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits / 8U + _size_bytes0_) <= capacity_bytes);
        int8_t _err0_ = reg_drone_srv_common_Heartbeat_0_1_serialize_(
            &obj->heartbeat, &buffer[offset_bits / 8U], &_size_bytes0_);
        if (_err0_ < 0)
        {
            return _err0_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        NUNAVUT_ASSERT((_size_bytes0_ * 8U) == 16ULL);
        offset_bits += _size_bytes0_ * 8U;  // Advance by the size of the nested object.
        NUNAVUT_ASSERT(offset_bits <= (capacity_bytes * 8U));
    }

    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad0_ > 0);
        const int8_t _err1_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err1_ < 0)
        {
            return _err1_;
        }
        offset_bits += _pad0_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }

    {   // uavcan.si.unit.temperature.Scalar.1.0[2] temperature_min_max
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 64ULL) <= (capacity_bytes * 8U));
        const size_t _origin0_ = offset_bits;
        {   // Array element #0
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits + 32ULL) <= (capacity_bytes * 8U));
            size_t _size_bytes1_ = 4UL;  // Nested object (max) size, in bytes.
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits / 8U + _size_bytes1_) <= capacity_bytes);
            int8_t _err2_ = uavcan_si_unit_temperature_Scalar_1_0_serialize_(
                &obj->temperature_min_max[0], &buffer[offset_bits / 8U], &_size_bytes1_);
            if (_err2_ < 0)
            {
                return _err2_;
            }
            // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
            NUNAVUT_ASSERT((_size_bytes1_ * 8U) == 32ULL);
            offset_bits += _size_bytes1_ * 8U;  // Advance by the size of the nested object.
            NUNAVUT_ASSERT(offset_bits <= (capacity_bytes * 8U));
        }
        {   // Array element #1
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits + 32ULL) <= (capacity_bytes * 8U));
            size_t _size_bytes2_ = 4UL;  // Nested object (max) size, in bytes.
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits / 8U + _size_bytes2_) <= capacity_bytes);
            int8_t _err3_ = uavcan_si_unit_temperature_Scalar_1_0_serialize_(
                &obj->temperature_min_max[1], &buffer[offset_bits / 8U], &_size_bytes2_);
            if (_err3_ < 0)
            {
                return _err3_;
            }
            // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
            NUNAVUT_ASSERT((_size_bytes2_ * 8U) == 32ULL);
            offset_bits += _size_bytes2_ * 8U;  // Advance by the size of the nested object.
            NUNAVUT_ASSERT(offset_bits <= (capacity_bytes * 8U));
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        NUNAVUT_ASSERT((offset_bits - _origin0_) == 64ULL);
        (void) _origin0_;
    }

    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad1_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad1_ > 0);
        const int8_t _err4_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad1_);  // Optimize?
        if (_err4_ < 0)
        {
            return _err4_;
        }
        offset_bits += _pad1_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }

    {   // uavcan.si.unit.voltage.Scalar.1.0[2] cell_voltage_min_max
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 64ULL) <= (capacity_bytes * 8U));
        const size_t _origin1_ = offset_bits;
        {   // Array element #0
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits + 32ULL) <= (capacity_bytes * 8U));
            size_t _size_bytes3_ = 4UL;  // Nested object (max) size, in bytes.
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits / 8U + _size_bytes3_) <= capacity_bytes);
            int8_t _err5_ = uavcan_si_unit_voltage_Scalar_1_0_serialize_(
                &obj->cell_voltage_min_max[0], &buffer[offset_bits / 8U], &_size_bytes3_);
            if (_err5_ < 0)
            {
                return _err5_;
            }
            // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
            NUNAVUT_ASSERT((_size_bytes3_ * 8U) == 32ULL);
            offset_bits += _size_bytes3_ * 8U;  // Advance by the size of the nested object.
            NUNAVUT_ASSERT(offset_bits <= (capacity_bytes * 8U));
        }
        {   // Array element #1
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits + 32ULL) <= (capacity_bytes * 8U));
            size_t _size_bytes4_ = 4UL;  // Nested object (max) size, in bytes.
            NUNAVUT_ASSERT(offset_bits % 8U == 0U);
            NUNAVUT_ASSERT((offset_bits / 8U + _size_bytes4_) <= capacity_bytes);
            int8_t _err6_ = uavcan_si_unit_voltage_Scalar_1_0_serialize_(
                &obj->cell_voltage_min_max[1], &buffer[offset_bits / 8U], &_size_bytes4_);
            if (_err6_ < 0)
            {
                return _err6_;
            }
            // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
            NUNAVUT_ASSERT((_size_bytes4_ * 8U) == 32ULL);
            offset_bits += _size_bytes4_ * 8U;  // Advance by the size of the nested object.
            NUNAVUT_ASSERT(offset_bits <= (capacity_bytes * 8U));
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        NUNAVUT_ASSERT((offset_bits - _origin1_) == 64ULL);
        (void) _origin1_;
    }

    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad2_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad2_ > 0);
        const int8_t _err7_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad2_);  // Optimize?
        if (_err7_ < 0)
        {
            return _err7_;
        }
        offset_bits += _pad2_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }

    {   // uavcan.si.unit.electric_charge.Scalar.1.0 available_charge
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 32ULL) <= (capacity_bytes * 8U));
        size_t _size_bytes5_ = 4UL;  // Nested object (max) size, in bytes.
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits / 8U + _size_bytes5_) <= capacity_bytes);
        int8_t _err8_ = uavcan_si_unit_electric_charge_Scalar_1_0_serialize_(
            &obj->available_charge, &buffer[offset_bits / 8U], &_size_bytes5_);
        if (_err8_ < 0)
        {
            return _err8_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        NUNAVUT_ASSERT((_size_bytes5_ * 8U) == 32ULL);
        offset_bits += _size_bytes5_ * 8U;  // Advance by the size of the nested object.
        NUNAVUT_ASSERT(offset_bits <= (capacity_bytes * 8U));
    }

    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad3_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad3_ > 0);
        const int8_t _err9_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad3_);  // Optimize?
        if (_err9_ < 0)
        {
            return _err9_;
        }
        offset_bits += _pad3_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }

    {   // reg.drone.srv.battery.Error.0.1 error
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        size_t _size_bytes6_ = 1UL;  // Nested object (max) size, in bytes.
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits / 8U + _size_bytes6_) <= capacity_bytes);
        int8_t _err10_ = reg_drone_srv_battery_Error_0_1_serialize_(
            &obj->_error, &buffer[offset_bits / 8U], &_size_bytes6_);
        if (_err10_ < 0)
        {
            return _err10_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        NUNAVUT_ASSERT((_size_bytes6_ * 8U) == 8ULL);
        offset_bits += _size_bytes6_ * 8U;  // Advance by the size of the nested object.
        NUNAVUT_ASSERT(offset_bits <= (capacity_bytes * 8U));
    }

    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad4_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad4_ > 0);
        const int8_t _err11_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad4_);  // Optimize?
        if (_err11_ < 0)
        {
            return _err11_;
        }
        offset_bits += _pad4_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }
    // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.

    NUNAVUT_ASSERT(offset_bits == 184ULL);

    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    *inout_buffer_size_bytes = (size_t) (offset_bits / 8U);

    return NUNAVUT_SUCCESS;
}

/// Deserialize an instance from the provided buffer.
/// The lifetime of the resulting object is independent of the original buffer.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original buffer where possible.
///
/// @param obj      The object to update from the provided serialized representation.
///
/// @param buffer   The source buffer containing the serialized representation. There are no alignment requirements.
///                 If the buffer is shorter or longer than expected, it will be implicitly zero-extended or truncated,
///                 respectively; see Specification for "implicit zero extension" and "implicit truncation" rules.
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the supplied serialized
///                                 representation, in bytes. Upon return this value will be updated with the
///                                 size of the consumed fragment of the serialized representation (in bytes),
///                                 which may be smaller due to the implicit truncation rule, but it is guaranteed
///                                 to never exceed the original buffer size even if the implicit zero extension rule
///                                 was activated. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_drone_srv_battery_Status_0_1_deserialize_(
    reg_drone_srv_battery_Status_0_1* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }

    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;

    // reg.drone.srv.common.Heartbeat.0.1 heartbeat
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    {
        size_t _size_bytes7_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        const int8_t _err12_ = reg_drone_srv_common_Heartbeat_0_1_deserialize_(
            &out_obj->heartbeat, &buffer[offset_bits / 8U], &_size_bytes7_);
        if (_err12_ < 0)
        {
            return _err12_;
        }
        offset_bits += _size_bytes7_ * 8U;  // Advance by the size of the nested serialized representation.
    }

    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.temperature.Scalar.1.0[2] temperature_min_max
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    // Array element #0
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    {
        size_t _size_bytes8_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        const int8_t _err13_ = uavcan_si_unit_temperature_Scalar_1_0_deserialize_(
            &out_obj->temperature_min_max[0], &buffer[offset_bits / 8U], &_size_bytes8_);
        if (_err13_ < 0)
        {
            return _err13_;
        }
        offset_bits += _size_bytes8_ * 8U;  // Advance by the size of the nested serialized representation.
    }
    // Array element #1
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    {
        size_t _size_bytes9_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        const int8_t _err14_ = uavcan_si_unit_temperature_Scalar_1_0_deserialize_(
            &out_obj->temperature_min_max[1], &buffer[offset_bits / 8U], &_size_bytes9_);
        if (_err14_ < 0)
        {
            return _err14_;
        }
        offset_bits += _size_bytes9_ * 8U;  // Advance by the size of the nested serialized representation.
    }

    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.voltage.Scalar.1.0[2] cell_voltage_min_max
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    // Array element #0
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    {
        size_t _size_bytes10_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        const int8_t _err15_ = uavcan_si_unit_voltage_Scalar_1_0_deserialize_(
            &out_obj->cell_voltage_min_max[0], &buffer[offset_bits / 8U], &_size_bytes10_);
        if (_err15_ < 0)
        {
            return _err15_;
        }
        offset_bits += _size_bytes10_ * 8U;  // Advance by the size of the nested serialized representation.
    }
    // Array element #1
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    {
        size_t _size_bytes11_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        const int8_t _err16_ = uavcan_si_unit_voltage_Scalar_1_0_deserialize_(
            &out_obj->cell_voltage_min_max[1], &buffer[offset_bits / 8U], &_size_bytes11_);
        if (_err16_ < 0)
        {
            return _err16_;
        }
        offset_bits += _size_bytes11_ * 8U;  // Advance by the size of the nested serialized representation.
    }

    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.electric_charge.Scalar.1.0 available_charge
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    {
        size_t _size_bytes12_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        const int8_t _err17_ = uavcan_si_unit_electric_charge_Scalar_1_0_deserialize_(
            &out_obj->available_charge, &buffer[offset_bits / 8U], &_size_bytes12_);
        if (_err17_ < 0)
        {
            return _err17_;
        }
        offset_bits += _size_bytes12_ * 8U;  // Advance by the size of the nested serialized representation.
    }

    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // reg.drone.srv.battery.Error.0.1 error
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    {
        size_t _size_bytes13_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        const int8_t _err18_ = reg_drone_srv_battery_Error_0_1_deserialize_(
            &out_obj->_error, &buffer[offset_bits / 8U], &_size_bytes13_);
        if (_err18_ < 0)
        {
            return _err18_;
        }
        offset_bits += _size_bytes13_ * 8U;  // Advance by the size of the nested serialized representation.
    }

    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);
    NUNAVUT_ASSERT(capacity_bytes >= *inout_buffer_size_bytes);

    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void reg_drone_srv_battery_Status_0_1_initialize_(reg_drone_srv_battery_Status_0_1* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = reg_drone_srv_battery_Status_0_1_deserialize_(out_obj, &buf, &size_bytes);
        NUNAVUT_ASSERT(err >= 0);
        (void) err;
    }
}

#ifdef __cplusplus
}
#endif
#endif // REG_DRONE_SRV_BATTERY_STATUS_0_1_INCLUDED_

