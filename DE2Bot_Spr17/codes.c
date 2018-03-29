 #include <stdio.h>

#define REV(x) (x * 0x0202020202ULL & 0x010884422010ULL) % 1023

int main(int argc, char *argv[]) {
    unsigned char a[] = {
        0x00, 0x02, 0x03, 0x09, 0x01, 0x02,
        0x42, 0x04, 0x05, 0x06, 0x07, 0x0C,
        0x0D, 0x0E, 0x0F, 0x1C, 0x1D, 0x5C,
        0xFF, 0x12, 0x14, 0x13, 0x11, 0x10, 0x15 };
        
    for (int i = 0; i < sizeof(a) / sizeof(a[0]); i++) {
        printf("DW &H%04llX\n", (REV(a[i]) << 8) | REV((unsigned char) ~a[i]));
    }
    return 0;
}