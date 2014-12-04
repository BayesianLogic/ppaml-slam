# slam_csv -- evaluates SLAM data set in relation to the ground truth
# Copyright (C) 2014  Galois, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#   1. Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in
#      the documentation and/or other materials provided with the
#      distribution.
#   3. Neither Galois's name nor the names of other contributors may be
#      used to endorse or promote products derived from this software
#      without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GALOIS AND OTHER CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL GALOIS OR OTHER
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import csv
import os

def read_floats_csv_file(path, filename, cols):
    """
      takes in path filename and expected number of collumns for csv
      returns a list of float-lists, and removes column labels
    """
    # TODO : move into utilities
    try:
        with open(os.path.join(path, filename), 'rb') as raw:
            sample = raw.read(1024)
            raw.seek(0)
            
            dialect = csv.Sniffer().sniff(sample)
            reader = csv.reader(raw, dialect)
            
            # removes header and provides file format check
            firstline = reader.next()
            assert(len(firstline) == cols)
            
            if not csv.Sniffer().has_header(sample):
                raw.seek(0) # reset reader
                
            processed = [map(float, line) for line in reader]
            
            return processed
    except Exception as e:
        print e, "  ::  ", os.path.join(path,filename)
        raise e

