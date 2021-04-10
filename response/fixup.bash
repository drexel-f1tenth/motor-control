#!/bin/bash

for f in ./*.txt; do
  output_file="$(echo "${f}" | awk -F '[-.]' '{print "log-" ($2 / 0.6) ".csv.new"}')"
  echo "setpoint,RPS,adjust,acc_x,acc_y" >"${output_file}"
  cat "${f}" |
    sed 's/^\[.*\]: //g' |
    sed 'N;s/\nIMU: /,/g' |
    sed 's/RPS: //g' >>"${output_file}"

  rm "${f}"
  mv "${output_file}" "${output_file%.*}"
done
