#!/bin/bash
if [[ !(-d src) ]]; then
  cd ..
fi

if [[ !(-d src) ]]; then
  echo "Please run this script in your homework folder."
  exit 1
fi

if [ $# -eq 1 ]; then
  echo "Packing for $1"
  zip -qr9 "hw1_$1.zip" include src
else
  echo "Usage: $0 <student_id>"
fi