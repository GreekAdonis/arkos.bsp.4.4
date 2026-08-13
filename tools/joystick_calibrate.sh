#!/bin/bash

TUNING_NODE=/sys/devices/platform/odroidgo3-joypad/joypad_tuning
FUZZ_NODE=/sys/devices/platform/odroidgo3-joypad/adc_fuzz
FLAT_NODE=/sys/devices/platform/odroidgo3-joypad/adc_flat
DEADZONE_NODE=/sys/devices/platform/odroidgo3-joypad/adc_deadzone

echo "=========================================="
echo "  ADC Joystick Calibration Tool"
echo "=========================================="
echo

# Check if tuning node exists
if [ ! -f "$TUNING_NODE" ]; then
    echo "Error: $TUNING_NODE not found"
    echo "Make sure JOYPAD_DEBUG_TUNING=1 in driver and kernel is recompiled"
    exit 1
fi

# Check if writable nodes exist
HAS_WRITABLE=0
if [ -w "$FUZZ_NODE" ] && [ -w "$FLAT_NODE" ] && [ -w "$DEADZONE_NODE" ]; then
    HAS_WRITABLE=1
fi

echo "=========================================="
echo "  Phase 1: Noise Detection"
echo "=========================================="
echo
echo "Please keep both sticks completely still!"
echo "Sampling for 3 seconds..."
echo

declare -A noise_min
declare -A noise_max
declare -A noise_sum
declare -A noise_count
declare -A cal

# Initialize
for axis in x y rx ry; do
    noise_min[$axis]=99999
    noise_max[$axis]=-99999
    noise_sum[$axis]=0
    noise_count[$axis]=0
done

# Sample noise for 5 seconds (50 samples)
for i in $(seq 1 50); do
    DATA=$(cat $TUNING_NODE)

    while read line; do
        axis=$(echo $line | cut -d: -f1)
        raw=$(echo $line | sed -n 's/.*raw=\([0-9]*\).*/\1/p')
        center=$(echo $line | sed -n 's/.*cal=\([0-9]*\).*/\1/p')

        if [[ -n "$raw" && -n "${noise_min[$axis]}" ]]; then
            if [ $raw -lt ${noise_min[$axis]} ]; then
                noise_min[$axis]=$raw
            fi
            if [ $raw -gt ${noise_max[$axis]} ]; then
                noise_max[$axis]=$raw
            fi
            noise_sum[$axis]=$(( ${noise_sum[$axis]} + raw ))
            noise_count[$axis]=$(( ${noise_count[$axis]} + 1 ))
            cal[$axis]=$center
        fi
    done <<< "$DATA"

    sleep 0.1
done

echo
echo "===== Noise Analysis ====="
echo

declare -A noise_range
declare -A noise_avg_dev
max_noise=0

for axis in x y rx ry; do
    if [[ -z "${cal[$axis]}" || ${noise_count[$axis]} -eq 0 ]]; then
        continue
    fi

    center=${cal[$axis]}
    noise_range[$axis]=$(( ${noise_max[$axis]} - ${noise_min[$axis]} ))

    # Calculate average deviation from center
    avg=0
    if [ ${noise_count[$axis]} -gt 0 ]; then
        avg=$(( ${noise_sum[$axis]} / ${noise_count[$axis]} ))
    fi
    dev_from_center=$(( avg - center ))
    if [ $dev_from_center -lt 0 ]; then
        dev_from_center=$(( -dev_from_center ))
    fi
    noise_avg_dev[$axis]=$dev_from_center

    echo "  $axis: center=$center, min=${noise_min[$axis]}, max=${noise_max[$axis]}"
    echo "      noise_range=${noise_range[$axis]}, avg_dev=${noise_avg_dev[$axis]}"

    # Track maximum noise
    if [ ${noise_range[$axis]} -gt $max_noise ]; then
        max_noise=${noise_range[$axis]}
    fi
done

# Check if noise is too high (possible drift or hardware issue)
if [ $max_noise -gt 200 ]; then
    echo
    echo "WARNING: Noise range is very high ($max_noise)!"
    echo "This may indicate:"
    echo "  - Joystick was moved during sampling"
    echo "  - ADC drift or hardware issue"
    echo "  - Poor connection"
    echo
    echo "Recommendation: Re-run calibration with sticks perfectly still"
    echo
fi

# Calculate recommended values
# fuzz = noise_range + margin (filter noise in event stream)
# flat = noise_range + margin (center area report as 0)
# deadzone = larger value for stable center
recommended_fuzz=$(( max_noise + 16 ))
recommended_flat=$(( max_noise + 16 ))
recommended_deadzone=$(( max_noise * 2 + 32 ))

# Ensure minimum values
if [ $recommended_fuzz -lt 32 ]; then
    recommended_fuzz=32
fi
if [ $recommended_flat -lt 32 ]; then
    recommended_flat=32
fi
if [ $recommended_deadzone -lt 64 ]; then
    recommended_deadzone=64
fi

echo
echo "===== Recommended Noise Settings ====="
echo
echo "  Max noise range across all axes: $max_noise"
echo
echo "  Recommended values:"
echo "    adc_fuzz    = $recommended_fuzz"
echo "    adc_flat    = $recommended_flat"
echo "    adc_deadzone = $recommended_deadzone"
echo

# Apply noise settings if writable
if [ $HAS_WRITABLE -eq 1 ]; then
    echo "Applying noise settings..."
    echo $recommended_fuzz > $FUZZ_NODE
    echo $recommended_flat > $FLAT_NODE
    echo $recommended_deadzone > $DEADZONE_NODE
    echo "  Applied: fuzz=$recommended_fuzz, flat=$recommended_flat, deadzone=$recommended_deadzone"
else
    echo "Note: fuzz/flat/deadzone are read-only (JOYPAD_DEBUG_TUNING=0)"
    echo "Add these to your DTS:"
    echo "    button-adc-fuzz = <$recommended_fuzz>;"
    echo "    button-adc-flat = <$recommended_flat>;"
    echo "    button-adc-deadzone = <$recommended_deadzone>;"
fi

echo
echo "=========================================="
echo "  Phase 2: Stick Range Calibration"
echo "=========================================="
echo
echo "Release sticks, then press ENTER to continue..."
read

echo
echo "Center calibration:"
cat $TUNING_NODE

declare -A min
declare -A max
declare -A tuning_p
declare -A tuning_n

# Read current DTS tuning values
while read line; do
    axis=$(echo $line | cut -d: -f1)
    p=$(echo $line | sed -n 's/.*tuning_p=\([0-9]*\).*/\1/p')
    n=$(echo $line | sed -n 's/.*tuning_n=\([0-9]*\).*/\1/p')

    if [[ -n "$p" && -n "$n" ]]; then
        tuning_p[$axis]=$p
        tuning_n[$axis]=$n
    fi
done < <(cat $TUNING_NODE)

# Initialize min/max for available axes
for axis in x y rx ry; do
    if [[ -n "${tuning_p[$axis]}" ]]; then
        min[$axis]=99999
        max[$axis]=-99999
    fi
done

echo
echo "Start calibration!"
echo "Rotate both sticks full range slowly."
echo "Time: 5s"
echo

# Sample 50 times (50 * 100ms = 5 seconds)
for i in $(seq 1 50); do
    DATA=$(cat $TUNING_NODE)

    while read line; do
        axis=$(echo $line | cut -d: -f1)
        raw=$(echo $line | sed -n 's/.*raw=\([0-9]*\).*/\1/p')
        center=$(echo $line | sed -n 's/.*cal=\([0-9]*\).*/\1/p')

        if [[ -n "$raw" && -n "${min[$axis]}" ]]; then
            if [ $raw -lt ${min[$axis]} ]; then
                min[$axis]=$raw
            fi
            if [ $raw -gt ${max[$axis]} ]; then
                max[$axis]=$raw
            fi
            cal[$axis]=$center
        fi
    done <<< "$DATA"

    sleep 0.1
done

echo
echo "===== Tuning Calibration Result ====="
echo

# Calculate and output tuning values
for axis in x y rx ry; do
    # Skip if axis not available
    if [[ -z "${cal[$axis]}" || -z "${min[$axis]}" ]]; then
        continue
    fi

    center=${cal[$axis]}
    positive_range=$(( ${max[$axis]} - center ))
    negative_range=$(( center - ${min[$axis]} ))

    # Avoid division by zero
    if [ $positive_range -le 0 ]; then
        echo "Warning: $axis positive_range=$positive_range, skipping"
        continue
    fi
    if [ $negative_range -le 0 ]; then
        echo "Warning: $axis negative_range=$negative_range, skipping"
        continue
    fi

    # tuning = 180000 / range
    new_p=$(( 180000 / positive_range ))
    new_n=$(( 180000 / negative_range ))

    echo "abs_${axis}-p-tuning = <$new_p>;"
    echo "abs_${axis}-n-tuning = <$new_n>;"
    echo "  center=$center, min=${min[$axis]}, max=${max[$axis]}"
    echo "  positive_range=$positive_range, negative_range=$negative_range"
    echo
done

echo
echo "===== Apply to running driver ====="
echo

for axis in x y rx ry; do
    if [[ -z "${cal[$axis]}" || -z "${min[$axis]}" ]]; then
        continue
    fi

    center=${cal[$axis]}
    positive_range=$(( ${max[$axis]} - center ))
    negative_range=$(( center - ${min[$axis]} ))

    if [ $positive_range -le 0 ] || [ $negative_range -le 0 ]; then
        continue
    fi

    new_p=$(( 180000 / positive_range ))
    new_n=$(( 180000 / negative_range ))

    echo "${axis}_p $new_p" > $TUNING_NODE
    echo "${axis}_n $new_n" > $TUNING_NODE
    echo "Applied: ${axis}_p=$new_p, ${axis}_n=$new_n"
done

echo
echo "=========================================="
echo "  Summary"
echo "=========================================="
echo
echo "===== Noise Settings ====="
if [ $HAS_WRITABLE -eq 1 ]; then
    echo "  fuzz=$recommended_fuzz, flat=$recommended_flat, deadzone=$recommended_deadzone"
else
    echo "  DTS config:"
    echo "    button-adc-fuzz = <$recommended_fuzz>;"
    echo "    button-adc-flat = <$recommended_flat>;"
    echo "    button-adc-deadzone = <$recommended_deadzone>;"
fi

echo
echo "===== Tuning Values ====="
for axis in x y rx ry; do
    if [[ -z "${cal[$axis]}" || -z "${min[$axis]}" ]]; then
        continue
    fi

    center=${cal[$axis]}
    positive_range=$(( ${max[$axis]} - center ))
    negative_range=$(( center - ${min[$axis]} ))

    if [ $positive_range -le 0 ] || [ $negative_range -le 0 ]; then
        continue
    fi

    new_p=$(( 180000 / positive_range ))
    new_n=$(( 180000 / negative_range ))

    echo "  abs_${axis}-p-tuning = <$new_p>;"
    echo "  abs_${axis}-n-tuning = <$new_n>;"
done

echo
echo "===== Current Status ====="
cat $TUNING_NODE
if [ $HAS_WRITABLE -eq 1 ]; then
    echo "fuzz=$(cat $FUZZ_NODE), flat=$(cat $FLAT_NODE), deadzone=$(cat $DEADZONE_NODE)"
fi

echo
echo "=========================================="
echo "  Tips"
echo "=========================================="
echo
echo "If you experience issues:"
echo "  - Stick drift/noise: Increase fuzz and flat values"
echo "  - Slow response: Decrease deadzone value"
echo "  - Dead zone too large: Decrease deadzone value"
echo "  - Not reaching full range: Re-run tuning calibration"
echo
echo "Typical values for good joysticks:"
echo "  fuzz=32-64, flat=32-64, deadzone=64-128"
echo
echo "If noise range > 100, consider:"
echo "  - Re-calibrate with sticks perfectly still"
echo "  - Check hardware connections"
echo "  - ADC may have drift issues"
echo
echo "Done."
