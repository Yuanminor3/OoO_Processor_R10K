# ------------------------------------------------------------------------------
# This script extracts performance metrics from VCS simulation .out files
# in the ./output directory. It parses cycles, instructions, CPI, and computes
# TPI = CPI × Clock_Period (provided as argument, e.g., sh get_cpi.sh 15.0).
# Results are saved to cpi_report.csv.
# If instructions = 0, CPI and TPI are set to 0.000000.
# A final 'Average' row summarizes total and average values.
# Run `make nuke` to delete the generated CSV file.
# ------------------------------------------------------------------------------

#!/bin/bash

#******** Usage: sh get_cpi.sh 15.0 ********
#******** Clock period must be given as a single-digit decimal (e.g., 15.0) *******

# Validate argument: must be like 15.0, 12.3, etc.
if [[ ! "$1" =~ ^[0-9]+[.][0-9]$ ]]; then
    echo "❌ Error: You must provide a clock period as a single-digit decimal (e.g., 15.0)"
    echo "✅ Usage: sh get_cpi.sh 15.0"
    exit 1
fi

CLK_PERIOD="$1"   # Use provided clock period
OUTPUT_DIR="./output"  # Directory containing .out files
CSV_FILE="cpi_report.csv"

# CSV Header
echo "Program,Cycles,Instructions,CPI,TPI(ns)" > "$CSV_FILE"

total_cycles=0
total_instrs=0

for file in "$OUTPUT_DIR"/*.out; do
    program=$(basename "$file" .out)

    # find line includes "cycles /" (11th lines from the end)
    line=$(tail -n 20 "$file" | grep "cycles /")

    cycles=$(echo "$line" | sed -E 's/.*@@[[:space:]]+([0-9]+)[[:space:]]+cycles.*/\1/')
    instrs=$(echo "$line" | sed -E 's/.*\/[[:space:]]*([0-9]+)[[:space:]]*instrs.*/\1/')
    cpi=$(echo "$line" | sed -E 's/.*= *([0-9.]+|inf)[[:space:]]*CPI.*/\1/')

    if [[ "$cycles" =~ ^[0-9]+$ ]] && [[ "$instrs" =~ ^[0-9]+$ ]]; then
        if [[ "$instrs" -eq 0 ]]; then
            cpi="0.000000"
            tpi="0.000000"
        else
            tpi=$(awk "BEGIN {printf \"%.6f\", $cpi * $CLK_PERIOD}")
        fi

        echo "$program,$cycles,$instrs,$cpi,$tpi" >> "$CSV_FILE"

        total_cycles=$((total_cycles + cycles))
        total_instrs=$((total_instrs + instrs))
    else
        echo "⚠️  Warning: Skipping $file due to parse error (line: $line)"
    fi
done

if [[ $total_instrs -ne 0 ]]; then
    avg_cpi=$(awk "BEGIN {printf \"%.6f\", $total_cycles / $total_instrs}")
    avg_tpi=$(awk "BEGIN {printf \"%.6f\", $avg_cpi * $CLK_PERIOD}")
else
    avg_cpi="0.000000"
    avg_tpi="0.000000"
fi

echo "Average,$total_cycles,$total_instrs,$avg_cpi,$avg_tpi" >> "$CSV_FILE"
echo "✅ Done. Output saved to $CSV_FILE"
