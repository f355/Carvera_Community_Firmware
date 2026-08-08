/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STREAMOUTPUTPOOL_H
#define STREAMOUTPUTPOOL_H

using namespace std;
#include <array>
#include <string>
#include <cstdio>
#include <cstdarg>

#include "libs/StreamOutput.h"

class StreamOutputPool : public StreamOutput {

public:
    StreamOutputPool(){
    }

    int puts(const char* s, int size)
    {
        int r = 0;
        for (StreamOutput *stream : this->streams) {
            if (stream == nullptr)
                continue;
            int k;
            if (communication_protocol == PROTOCOL_SMOOTHIE) {
                k = stream->puts(s);
            }
            else {
                k = stream->puts(s,size);
            }
            if (k > r)
                r = k;
        }
        return r;
    }

    void append_stream(StreamOutput* stream)
    {
        for (StreamOutput *existing : this->streams)
            if (existing == stream)
                return;
        for (StreamOutput *&entry : this->streams) {
            if (entry == nullptr) {
                entry = stream;
                return;
            }
        }
    }

    void remove_stream(StreamOutput* stream)
    {
        for (StreamOutput *&entry : this->streams)
            if (entry == stream)
                entry = nullptr;
    }

    bool frames_protocol_output() const { return true; }

    void on_protocol_changed()
    {
        for (StreamOutput *stream : this->streams)
            if (stream != nullptr)
                stream->on_protocol_changed();
    }

private:
    std::array<StreamOutput*, 2> streams{};
};

#endif
