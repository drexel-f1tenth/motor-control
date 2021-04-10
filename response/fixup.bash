#!/bin/bash

for f in ./*.txt; do
  output_num="${f##*-}"
  output_num="${output_num%%.*}"
  output_num="$(echo "${output_num} / 0.6" | bc)"
  # new extension is used to avoid file name conflicts when renaming
  output_file="log-${output_num}.csv.new"
  echo "setpoint,RPS,adjust,acc_x,acc_y" >"${output_file}"
  cat "${f}" |
    sed 's/^\[.*\]: //g' |
    sed 'N;s/\nIMU: /,/g' |
    sed 's/RPS: //g' >>"${output_file}"

  rm "${f}"
  mv "${output_file}" "${output_file%.*}"
done
