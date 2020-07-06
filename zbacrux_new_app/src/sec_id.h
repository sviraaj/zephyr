#ifndef __SEC_ID_H__
#define __SEC_ID_H__

#include <string.h>
#include <stdio.h>

static void string2hexString(char* input, char* output)
{
    int loop;
    int i; 
    
    i=0;
    loop=0;
    
    while(input[loop] != '\0')
    {
        sprintf((char*)(output+i),"%02X", input[loop]);
        loop+=1;
        i+=2;
    }
    //insert NULL at the end of the output string
    output[i++] = '\0';
}

static inline void mutate_id_secret(char* id, char* hash)
{
#if 0
    char hashin[64];

    for (int i = 0; i < strlen(id); i++) {
        char c = id[i]; 
        hashin[i] = ((hashin[i])) + c; 
        if (i >= 2)
        {
            hashin[i] += (hashin[i-1] & hashin[i-2]);
        }
    }

    string2hexString(hashin, hash);
#endif
    strcpy(hash, id);
}

#endif
