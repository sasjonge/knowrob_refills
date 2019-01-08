#!/bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
DATA_DIR=$DIR/../data
MODEL_DIR=$DIR/../../refills_models/models/
OUTPUT_DIR=$DIR/../owl

rosrun knowrob_refills import_catalog.py \
    --catalog \
    --datafile=$MODEL_DIR/model_data.yaml \
    -i $DATA_DIR/tax.xlsx:Tabelle1 \
    -i $DATA_DIR/gtin.xlsx:DAN-GTIN \
    -o $OUTPUT_DIR
