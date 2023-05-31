# Copyright [2023] fjrodl

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import re
import sys


import getopt

def parse_line(line):
    res = {}
    if line.startswith("["):
        complete_message = line.split(":",1)
        context = complete_message[0].split()

        res = re.findall(r'\[.*?\]', complete_message[0])
        
        # Obtener los valores entre los corchetes
        field1 = res[0].strip('[').strip(']').strip()
        field2 = res[1].strip('[').strip(']').strip()
        field3 = res[2].strip('[').strip(']').strip()
        
        # Crear un diccionario con los campos y el texto
        json_data = {
            'RealTime'      : '-',
            'Debug_level'   : field1,
            'Timestamp'     : field2,
            'Package'       : field3,
            'Function'      : '-',
            'Line'          : '-',
            'Name'          : '-',
            'Message'       :  complete_message[1]
        }
        return json_data

def convert_lines_to_json(file_path):
    result = []
    
    # Abrir el archivo en modo lectura
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
        # Iterar sobre las líneas del archivo
        for line in lines:
            json_data = parse_line(line)
            result.append(json_data)
    
    return result



def main(argv):
    inputfile = ''
    outputfile = ''
    opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    for opt, arg in opts:
        if opt == '-h':
            print ('log2json_from_file.py -i <inputfile> [-o <outputfile>]')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
    # print ('Input file is ', inputfile)
    # print ('Output file is ', outputfile)



    # Convertir las líneas al formato JSON
    json_data = convert_lines_to_json(inputfile)

    # Imprimir los datos en formato JSON
    print(json.dumps(json_data, indent=4))

    if outputfile != '':
        with open(outputfile, 'w', encoding='utf-8') as f:
            json.dump(json_data, f, ensure_ascii=False, indent=4)


if __name__ == "__main__":
   main(sys.argv[1:])
   