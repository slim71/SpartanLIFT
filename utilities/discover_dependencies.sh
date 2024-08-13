#!/bin/bash

# List Debian packages that a given executable is dependent on
BINARY_FILE=$1

LIB_LIST=$(ldd $1 | gawk '{ print $1 }' | tr '\n' ' ')

for THIS_LIB in $LIB_LIST; do
  dpkg-query -S $THIS_LIB
done
