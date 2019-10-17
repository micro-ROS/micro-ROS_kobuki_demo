// Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Vector3.h"

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <stdio.h> //printf
#include <string.h> //strcmp
#include <stdlib.h> //atoi

#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>

#define STREAM_HISTORY  8
#define BUFFER_SIZE     UXR_CONFIG_UDP_TRANSPORT_MTU * STREAM_HISTORY

int main(int args, char** argv)
{
    // CLI
    if(3 > args || 0 == atoi(argv[2]))
    {
        printf("usage: program [-h | --help] | ip port [<max_topics>]\n");
        return 0;
    }

    char* ip = argv[1];
    uint16_t port = (uint16_t)atoi(argv[2]);
    uint32_t max_topics = (args == 4) ? (uint32_t)atoi(argv[3]) : UINT32_MAX;

    // Transport
    uxrUDPTransport transport;
    uxrUDPPlatform udp_platform;
    if(!uxr_init_udp_transport(&transport, &udp_platform, ip, port))
    {
        printf("Error at create transport.\n");
        return 1;
    }

    // Session
    uxrSession session;
    uxr_init_session(&session, &transport.comm, 0xAAAABCCB);
    if(!uxr_create_session(&session))
    {
        printf("Error at create session.\n");
        return 1;
    }

    // Streams Reliable
    uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
    uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    uint8_t input_reliable_stream_buffer[BUFFER_SIZE];
    uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);


    // Create entities
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
                                      "<participant>"
                                          "<rtps>"
                                              "<name>microteleop_attiude</name>"
                                          "</rtps>"
                                      "</participant>"

                                  "</dds>";
    uint16_t participant_req = uxr_buffer_create_participant_xml(&session, reliable_out, participant_id, 0, participant_xml, UXR_REPLACE);

    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* topic_xml = "<dds>"
                                "<topic>"
                                    "<name>rt/drone/robot_pose</name>"
                                    "<dataType>geometry_msgs::msg::dds_::Vector3_</dataType>"
                                "</topic>"
                            "</dds>";
    uint16_t topic_req = uxr_buffer_create_topic_xml(&session, reliable_out, topic_id, participant_id, topic_xml, UXR_REPLACE);

    uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* publisher_xml = "";
    uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session, reliable_out, publisher_id, participant_id, publisher_xml, UXR_REPLACE);

    uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    const char* datawriter_xml = "<dds>"
                                     "<data_writer>"
                                         "<topic>"
                                             "<kind>NO_KEY</kind>"
                                             "<name>rt/drone/robot_pose</name>"
                                            "<dataType>geometry_msgs::msg::dds_::Vector3_</dataType>"
                                         "</topic>"
                                     "</data_writer>"
                                 "</dds>";
    uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(&session, reliable_out, datawriter_id, publisher_id, datawriter_xml, UXR_REPLACE);

    // Send create entities message and wait its status
    uint8_t status[4];
    uint16_t requests[4] = {participant_req, topic_req, publisher_req, datawriter_req};
    if(!uxr_run_session_until_all_status(&session, 1000, requests, status, 4))
    {
        printf("Error at create entities: participant: %i topic: %i publisher: %i darawriter: %i\n", status[0], status[1], status[2], status[3]);
        return 1;
    }

    // Write topics
    bool connected = true;
    uint32_t count = 0;

    double pitch = 0.0;
    double roll = 0.0;
    double yaw = 0.0;

    double step = 0.1;
    
    fd_set readfds;
    int    num_readable;
    struct timeval tv;
    int    num_bytes;
    char   buf[2];
    int    fd_stdin;

    fd_stdin = fileno(stdin);


    system ("/bin/stty raw");

    while(connected && count < max_topics)
    {   
        FD_ZERO(&readfds);
        FD_SET(fileno(stdin), &readfds);

        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        fflush(stdout);

        num_readable = select(fd_stdin + 1, &readfds, NULL, NULL, &tv);

        if (num_readable == 0) {
            // SEND COMMAND
            Vector3 cmd = {pitch,roll,yaw};

            ucdrBuffer ub;
            uint32_t topic_size = Vector3_size_of_topic(&cmd,0);
            uxr_prepare_output_stream(&session, reliable_out, datawriter_id, &ub, topic_size);
            Vector3_serialize_topic(&ub,&cmd);

            printf("\rSend Vector3 on rt/drone/robot_pose: pitch: %f, roll: %f, yaw: %f \n", pitch, roll, yaw);

            connected = uxr_run_session_time(&session, 50);
        } else {
            num_bytes = read(fd_stdin, buf, 1);
            // if (num_bytes < 0) {
            //         fprintf(stderr, "\nError on read : %s\n", strerror(errno));
            //         exit(1);
            // }
            /* process command, maybe by sscanf */
            printf("\rRead %c \n", buf[0]);

            switch (buf[0])
            {
            case 'i':
                pitch += step;
                break;
            case 'o':
                pitch -= step;
                break;
            case 'k':
                roll += step;
                break;
            case 'l':
                roll -= step;
                break;
            case ',':
                yaw += step;
                break;
            case '.':
                yaw -= step;
                break;                          
            case '+':
                step += 0.1;
                break;
            case '-':
                step -= 0.1;
                break;
            case 'x':
                goto end;
                break;
            }
            
            buf[0] = '\0';
        }
    }

end:   

    system ("/bin/stty echo");

    // Delete resources
    uxr_delete_session(&session);
    uxr_close_udp_transport(&transport);

    return 0;
}
