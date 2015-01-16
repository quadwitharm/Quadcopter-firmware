#include "shell/b64.h"

static const char cb64[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static const char cd64[]={62,-1,-1,-1,63,52,53,54,55,56,57,58,59,60,61,-1,-1,-1,-2/* = */,-1,-1,-1,
                0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,-1,-1,-1,
                -1,-1,-1,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51};

static void encodeblock(uint8_t *in,uint8_t *out,int len ){
    out[0] = (uint8_t) cb64[ (uint16_t)(in[0] >> 2) ];
    out[1] = (uint8_t) cb64[ (uint16_t)(((in[0] & 0x03) << 4) | ((in[1] & 0xf0) >> 4)) ];
    out[2] = (uint8_t) (len > 1 ? cb64[ (uint16_t)(((in[1] & 0x0f) << 2) | ((in[2] & 0xc0) >> 6)) ] : '=');
    out[3] = (uint8_t) (len > 2 ? cb64[ (uint16_t)(in[2] & 0x3f) ] : '=');
}

void b64Encode(uint8_t *in,uint8_t *out,int len){
    uint8_t inbuf[3];
    uint8_t outbuf[4];

    int index_in = 0;
    int index_out = 0;

    int i=0, l=0;

    *inbuf = (uint8_t) 0;
    *outbuf = (uint8_t) 0;

    while(index_in < len){
        l = 0;
        for(i=0;i<3;i++){            
            if(index_in < len){
                inbuf[i] = in[index_in++];
                l++;
            }else{
                inbuf[i] = (uint8_t)0;
            }
        }
        if( l > 0 ){
            encodeblock(inbuf,outbuf,l);
            for( i = 0; i < 4; i++ ){
                out[index_out++] = outbuf[i];
            }
        }
    }
}


static void decodeblock( uint8_t *in, uint8_t *out ){   
    out[ 0 ] = (uint8_t ) (in[0] << 2 | in[1] >> 4);
    out[ 1 ] = (uint8_t ) (in[1] << 4 | in[2] >> 2);
    out[ 2 ] = (uint8_t ) (((in[2] << 6) & 0xc0) | in[3]);
}

bool b64Decode(uint8_t *in,uint8_t *out,int len){
    uint8_t inbuf[4];
    uint8_t outbuf[3];

    int index_in = 0;
    int index_out = 0;

    int8_t v = 0;
    int i=0, l=0;

    *inbuf = (uint8_t) 0;
    *outbuf = (uint8_t) 0;

    while(index_in < len){
        for( l = 0, i = 0; i < 4 && index_in < len; i++ ){
            v = in[index_in++];
            v = ((v < 43 /* + */ || v > 122 /* z */) ? -1 :  cd64[ v - 43 ]);
            if(v == -1){
                return false;
            }else if( v >= 0 ) {
                l++;
                inbuf[ i ] = (uint8_t) v;
            }else if(v == -2){
                inbuf[i] = (uint8_t) 0;
            }
        }
        if(l > 0){
            decodeblock( inbuf, outbuf );
            for( i = 0; i < l - 1; i++ ){
                out[index_out++] = outbuf[i];
            }    
        }
    }

    return true;
}

int getB64EncodeLen(int len){
    return ((len + ( (len % 3) ? (3 - (len % 3)) : 0) ) / 3) * 4;
}

int getB64DecodeLen(int len){
    return (len * 3) >> 2;
}





