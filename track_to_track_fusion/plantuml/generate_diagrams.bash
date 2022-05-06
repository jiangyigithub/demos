#!/bin/bash

for diagram in *.plantuml; do
  echo "Generating ${diagram}"
  plantuml ${diagram} -tsvg
done
sed 's/\r$//' -i *svg
