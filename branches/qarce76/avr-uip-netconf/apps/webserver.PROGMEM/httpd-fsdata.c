#include <avr/pgmspace.h>
static const char data_header_html[] PROGMEM = {
	/* /header.html */
	0x2f, 0x68, 0x65, 0x61, 0x64, 0x65, 0x72, 0x2e, 0x68, 0x74, 0x6d, 0x6c, 0,
	0x3c, 0x21, 0x44, 0x4f, 0x43, 0x54, 0x59, 0x50, 0x45, 0x20, 
	0x48, 0x54, 0x4d, 0x4c, 0x20, 0x50, 0x55, 0x42, 0x4c, 0x49, 
	0x43, 0x20, 0x22, 0x2d, 0x2f, 0x2f, 0x57, 0x33, 0x43, 0x2f, 
	0x2f, 0x44, 0x54, 0x44, 0x20, 0x48, 0x54, 0x4d, 0x4c, 0x20, 
	0x34, 0x2e, 0x30, 0x31, 0x20, 0x54, 0x72, 0x61, 0x6e, 0x73, 
	0x69, 0x74, 0x69, 0x6f, 0x6e, 0x61, 0x6c, 0x2f, 0x2f, 0x45, 
	0x4e, 0x22, 0x20, 0x22, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 
	0x2f, 0x77, 0x77, 0x77, 0x2e, 0x77, 0x33, 0x2e, 0x6f, 0x72, 
	0x67, 0x2f, 0x54, 0x52, 0x2f, 0x68, 0x74, 0x6d, 0x6c, 0x34, 
	0x2f, 0x6c, 0x6f, 0x6f, 0x73, 0x65, 0x2e, 0x64, 0x74, 0x64, 
	0x22, 0x3e, 0xa, 0x3c, 0x68, 0x74, 0x6d, 0x6c, 0x3e, 0xa, 
	0x20, 0x20, 0x3c, 0x68, 0x65, 0x61, 0x64, 0x3e, 0xa, 0x20, 
	0x20, 0x20, 0x20, 0x3c, 0x74, 0x69, 0x74, 0x6c, 0x65, 0x3e, 
	0x57, 0x65, 0x6c, 0x63, 0x6f, 0x6d, 0x65, 0x20, 0x74, 0x6f, 
	0x20, 0x74, 0x68, 0x65, 0x20, 0x75, 0x49, 0x50, 0x20, 0x77, 
	0x65, 0x62, 0x20, 0x73, 0x65, 0x72, 0x76, 0x65, 0x72, 0x21, 
	0x3c, 0x2f, 0x74, 0x69, 0x74, 0x6c, 0x65, 0x3e, 0xa, 0x20, 
	0x20, 0x20, 0x20, 0x3c, 0x6c, 0x69, 0x6e, 0x6b, 0x20, 0x72, 
	0x65, 0x6c, 0x3d, 0x22, 0x73, 0x74, 0x79, 0x6c, 0x65, 0x73, 
	0x68, 0x65, 0x65, 0x74, 0x22, 0x20, 0x74, 0x79, 0x70, 0x65, 
	0x3d, 0x22, 0x74, 0x65, 0x78, 0x74, 0x2f, 0x63, 0x73, 0x73, 
	0x22, 0x20, 0x68, 0x72, 0x65, 0x66, 0x3d, 0x22, 0x73, 0x74, 
	0x79, 0x6c, 0x65, 0x2e, 0x63, 0x73, 0x73, 0x22, 0x3e, 0x20, 
	0x20, 0xa, 0x20, 0x20, 0x3c, 0x2f, 0x68, 0x65, 0x61, 0x64, 
	0x3e, 0xa, 0xa, };

static const char data_devconf_html[] PROGMEM = {
	/* /devconf.html */
	0x2f, 0x64, 0x65, 0x76, 0x63, 0x6f, 0x6e, 0x66, 0x2e, 0x68, 0x74, 0x6d, 0x6c, 0,
	0x1f, 0x8b, 0x8, 0x8, 0x39, 0x5b, 0xe8, 0x4d, 00, 0x3, 
	0x64, 0x65, 0x76, 0x63, 0x6f, 0x6e, 0x66, 0x2e, 0x68, 0x74, 
	0x6d, 0x6c, 0x5f, 0x67, 0x7a, 00, 0xad, 0x57, 0x6d, 0x6f, 
	0xdb, 0x36, 0x10, 0xfe, 0x6c, 0xfd, 0xa, 0x4e, 0x5f, 0x24, 
	0xa3, 0x86, 0x9d, 0x61, 0xd8, 0x17, 0xbf, 0x1, 0xad, 0x5b, 
	0x64, 0x1, 0x96, 0xd4, 0x58, 0x82, 0x16, 0x43, 0x51, 0xc, 
	0xb4, 0x48, 0xcb, 0x4c, 0x64, 0x52, 0x10, 0x69, 0x3b, 0xda, 
	0xe0, 0xff, 0xbe, 0x3b, 0x52, 0x94, 0x25, 0xdb, 0x71, 0xd2, 
	0x61, 0x46, 0x2, 0x59, 0xe4, 0x73, 0xf7, 0x1c, 0xef, 0x1e, 
	0x1e, 0xe9, 0x20, 0x18, 0x2f, 0x14, 0x2b, 0x89, 0x92, 0x99, 
	0xa2, 0x6c, 0x12, 0x32, 0x35, 0xa7, 0x29, 0x8f, 0xbb, 0xa3, 
	0x70, 0x1a, 0x4, 0xc1, 0x58, 0x27, 0x85, 0xc8, 0xd, 0x31, 
	0x65, 0xce, 0x27, 0xa1, 0xe1, 0xcf, 0x66, 0xf0, 0x48, 0xb7, 
	0xd4, 0x8d, 0x22, 0x62, 0x30, 0x20, 0x5, 0x37, 0x9b, 0x42, 
	0x12, 0xb3, 0xe2, 0x64, 0x4b, 0xb3, 0xd, 0x27, 0x6a, 0x69, 
	0x5f, 0xa, 0xca, 0x84, 0x22, 0x8b, 0x8d, 0x31, 0xa, 0x67, 
	0xa9, 0x21, 0x42, 0x93, 0x64, 0xc5, 0x93, 0x27, 0xce, 0x1a, 
	0x76, 0x54, 0x12, 0xbe, 0xce, 0x4d, 0x49, 0xb4, 0x29, 0x84, 
	0x4c, 0x89, 0x58, 0x12, 0xa9, 0x24, 0x27, 0xb4, 0xe0, 0x1e, 
	0xdd, 0x23, 0xaa, 0x40, 0xb, 0xf0, 0x5a, 0xb8, 0x9, 0xa9, 
	0x5a, 0xee, 0x75, 0xb0, 0xdc, 0xc8, 0xc4, 0x8, 0x20, 0x4a, 
	0xb9, 0x99, 0x39, 0xab, 0x2f, 0x18, 0x4c, 0x6c, 0x61, 0x9f, 
	0x17, 0x8f, 0x5d, 0xf2, 0x4f, 0xd0, 0x11, 0xcb, 0xf8, 0xa7, 
	0x7a, 0x20, 0xe8, 0x74, 0xaa, 0x10, 0xc2, 0x70, 0x14, 0x74, 
	0xb6, 0xb4, 0x70, 0x3e, 0x7f, 0xe7, 0x32, 0x35, 0x2b, 0x32, 
	0x21, 0x1e, 0xd9, 0xcf, 0xec, 0xc8, 0xc8, 0xda, 0xb7, 0x20, 
	0x13, 0xb2, 0x91, 0x8c, 0x2f, 0x85, 0xe4, 0xc, 0xdd, 0xf9, 
	0x69, 0xb4, 0xa9, 0x42, 0xc7, 0x61, 0x4f, 0x53, 0xcf, 0xd9, 
	0x34, 0x81, 0xbb, 0xe, 0xcf, 0x34, 0x6f, 00, 0x6c, 0x1c, 
	0x4b, 0x55, 0xc4, 0x18, 0x8b, 0x80, 0x8, 0xae, 0x46, 0xf0, 
	0x18, 0x37, 0xc3, 0x82, 0x81, 0x77, 0xef, 0xec, 0x5a, 0x9a, 
	0x6c, 0xdf, 0xc4, 0xf7, 0x9a, 0xd0, 0xce, 0x1d, 0x53, 0x22, 
	0xa0, 0x66, 0xdd, 0x7, 0xf8, 0xd7, 0xe0, 0xdc, 0xdb, 0x3a, 
	0x6a, 0x6e, 0x4e, 0xeb, 0xb6, 0x13, 0xb0, 0x4e, 0x1c, 0x4d, 
	0xc5, 0x96, 0xcb, 0xaa, 0xc0, 0x54, 0x93, 0x5, 0xc7, 0x5a, 
	0x35, 0xaa, 0xc9, 0x14, 0x14, 0xc5, 0xac, 0xaa, 0xa, 0x5e, 
	0xa8, 0x14, 0x60, 0x1d, 0xa0, 0xe5, 0x92, 0x29, 0xae, 0xd1, 
	0x1, 0xe1, 0xcf, 0x42, 0x9b, 0x1e, 0xa1, 0x59, 0x76, 0x12, 
	0x8c, 0xb5, 0x45, 0x9f, 0x5, 0xb7, 0xb1, 0x2a, 0xc8, 0xbe, 
	0x8f, 0xa0, 0xae, 0xbf, 0x7e, 0xa1, 0xfe, 0x3d, 0x22, 0xf9, 
	0xce, 0xe, 0x5d, 0x50, 0xc2, 0xff, 0x20, 0x3, 0x9b, 0xfe, 
	0x63, 0x15, 0x80, 0x97, 0xb8, 0x5d, 0x7d, 0x34, 0xf2, 0x1, 
	0xf5, 0x8d, 0xba, 0xb7, 0xda, 0x8f, 0xbb, 0xdd, 0x51, 0x33, 
	0x96, 0xfd, 0xf, 0xa8, 0xe1, 0x8c, 0x14, 0xc0, 0x64, 0x49, 
	0x41, 0x61, 0xa3, 0x53, 0xb1, 0x5c, 0x8c, 0xa1, 0x52, 0xd0, 
	0x59, 0x87, 0xa6, 0x68, 0x68, 0x8, 0x74, 0x43, 0xe0, 0x53, 
	0xe7, 0xde, 0xa8, 0x34, 0xcd, 0xf8, 0x5f, 0x5b, 0xa1, 0xc5, 
	0x42, 0x64, 0xc2, 0x94, 0xb1, 0xb0, 0x9, 0x21, 0xee, 0x83, 
	0xeb, 00, 0x52, 0xa8, 0x75, 0xb2, 0x59, 0x73, 0x69, 0xfa, 
	0xb0, 0x57, 0x3f, 0x65, 0x1c, 0xbf, 0x7e, 0x28, 0x6f, 0x18, 
	0x82, 0x47, 0x1e, 0xb, 0xf1, 0xf2, 0xbe, 0x36, 0x65, 0xc6, 
	0xfb, 0x4c, 0xe8, 0x3c, 0xa3, 0x25, 0x46, 0x1b, 0x2d, 0x32, 
	0x95, 0x3c, 0x45, 0x5d, 0x8f, 0x82, 0xcf, 0x9, 0x8a, 0x44, 
	0xd8, 0x3e, 0xa2, 0xda, 0x93, 0xdd, 0x63, 0x17, 0xf1, 0xce, 
	0xa9, 0x33, 0xb0, 0x4b, 0x6a, 0x8a, 0xe9, 0xb, 0x2e, 0x26, 
	0xe3, 0x31, 0x2c, 0xea, 0xe6, 0x63, 0x5b, 0x44, 0x6f, 0x59, 
	0x93, 0x35, 0xab, 0x96, 0x75, 0x4a, 0xed, 0xbd, 0xe1, 0x7c, 
	0x9b, 0x59, 0xc8, 0x24, 0xdb, 0x30, 0x1e, 0x2f, 0x5, 0xa8, 
	0x8e, 0xae, 0x5b, 0x7c, 0x2b, 0x4e, 0xd9, 0x79, 0x4a, 0xfd, 
	0xa1, 0x7c, 0xa0, 0xe9, 0x1d, 0xe0, 0xe3, 0x8, 0x51, 0x51, 
	0xf7, 0xdb, 0xd5, 0xf7, 0x91, 0xab, 0x52, 0xd5, 0xc9, 0x1b, 
	0x76, 0x49, 0xc1, 0xa9, 0xe1, 0x95, 0x69, 0x1c, 0x39, 0x40, 
	0x54, 0x5, 0xeb, 0xde, 0xfa, 0xba, 0x48, 0x50, 0x46, 0x55, 
	0x14, 0xad, 0x29, 0x3c, 0x13, 0x30, 0x7d, 0x47, 0xa7, 0x42, 
	0x54, 0xf1, 0x21, 0x7f, 0x9f, 0xe6, 0x39, 0x97, 0x6c, 0xb6, 
	0x12, 0x19, 0x8b, 0xdd, 0x74, 0xd7, 0x2e, 0x75, 0x3c, 0x70, 
	0x6f, 0x70, 0x82, 0x8c, 0xef, 0x67, 0x7f, 0xdc, 0xcc, 0x1f, 
	0xc8, 0xc3, 0x9f, 0xf3, 0x4f, 0x67, 0x8f, 0x98, 0x46, 0x56, 
	0x98, 0xfa, 0xf8, 0xdb, 0x6c, 0x1e, 0x63, 0x32, 0x2c, 0x47, 
	0xbd, 0x14, 0xc9, 0xd, 0xd4, 0xaa, 0xcf, 0xf8, 0x56, 0xe4, 
	0x98, 0x5f, 0xa, 0x25, 0x3b, 0xc8, 0xf5, 0x5, 0xa8, 0x5c, 
	0xbf, 0x19, 0x9a, 0xee, 0xce, 0x41, 0xf7, 0xf0, 0xdf, 0xe, 
	0xef, 0xde, 0x50, 0x23, 0x92, 0xd8, 0x57, 0xeb, 0xd5, 0xf8, 
	0xaa, 0xfd, 0xf9, 0xa6, 00, 0x2f, 0x63, 0xdb, 0x11, 0xd6, 
	0xd8, 0xd3, 0x10, 0xef, 0xb8, 0xb9, 0xe7, 0x26, 0x16, 0xda, 
	0xc5, 0xda, 0x43, 0x91, 0xdf, 0xcc, 0xed, 0xe3, 0xee, 0xd6, 
	0x3e, 0xae, 0xbf, 0x1e, 0x36, 0x2d, 0xb4, 0xea, 0x1a, 0xda, 
	0xd8, 0xca, 0x8d, 0xa5, 0xd6, 0x1b, 0xed, 0xa4, 0xf5, 0x1e, 
	0x47, 0x9, 0x8f, 0xb5, 0x62, 0xbc, 0x47, 0x42, 0x6d, 0x6d, 
	0xc3, 0xee, 0xc8, 0x9b, 0x56, 0x3e, 0xf6, 0x76, 0xb7, 0xb6, 
	0x68, 0x5c, 0xc1, 0xff, 0xb, 0x9, 0x5b, 0x25, 0x79, 0x58, 
	0x5b, 0xee, 0x3, 0xef, 0xf1, 0x5c, 0x49, 0xaa, 0x7e, 0xe8, 
	0x72, 0x31, 0x7a, 0x19, 0xa, 0x15, 0x69, 0x40, 0xef, 0x6e, 
	0x2f, 0x40, 0xa1, 0x20, 0xd, 0xe8, 0xf5, 0x57, 0xf, 0x6d, 
	0xf4, 0x94, 0x88, 0x89, 0x2d, 0x18, 0x44, 0xbd, 0xba, 0xb1, 
	0x9d, 0x36, 0x2, 0x7f, 0x1f, 0xab, 0x92, 0x72, 0xce, 0xda, 
	0x35, 0x3c, 0x3c, 0x3a, 0x7c, 0xd7, 0x88, 0x80, 0x3f, 0x51, 
	0x72, 0xd9, 0x7f, 0xd4, 0x7e, 0x3f, 0x33, 0x35, 0x83, 0x1, 
	0x91, 0xc6, 0xdd, 0x5a, 0x16, 0x8d, 0x4d, 0x18, 0x8c, 0xc1, 
	0x19, 0x11, 0x78, 0xff, 0xb3, 0x4e, 0x43, 0x32, 0x5, 0xc4, 
	0x78, 0x51, 0x4c, 0x41, 0x2e, 0x3b, 0x55, 0x3c, 0x11, 0xd0, 
	0x8c, 0x81, 0xe3, 0x41, 0xf, 0x71, 0x1c, 0x4e, 0xa5, 0x35, 
	0xc1, 0x86, 0x30, 0x9, 0xdd, 0x82, 0x43, 0x42, 0xa8, 0x8d, 
	0x17, 0x1c, 0x54, 0xd4, 0x7a, 0x65, 0xd6, 0x59, 0x48, 0xd6, 
	0xdc, 0xac, 0x14, 0xf8, 0x85, 0x26, 0x15, 0x4e, 0x6d, 0x24, 
	0x37, 0x73, 0x72, 0xb, 0x15, 0x1a, 0xda, 0x97, 0xb1, 0x90, 
	0xf9, 0xc6, 0xdf, 0x2f, 0xed, 0x91, 0x13, 0x1e, 0x1c, 0x63, 
	0x21, 0x43, 0x77, 0x41, 0x98, 0xb8, 0x72, 0xc2, 0x35, 0x35, 
	0x59, 0x51, 0x99, 0xe2, 0xbb, 0x97, 0x46, 0x48, 0x6, 0x53, 
	0x82, 0xdf, 0x7f, 0xd0, 0x61, 0x25, 0xc2, 0x96, 0xcb, 0x5a, 
	0xd4, 0xd6, 0xa9, 0x7b, 0xb, 0xbc, 0x3e, 0x21, 0x1d, 0x83, 
	0x29, 0x44, 0xff, 0x9e, 0x31, 0xb8, 0x81, 0xe8, 0x61, 0x8b, 
	0xa, 0x1b, 0x97, 0x67, 0xb2, 0x9a, 0xa, 0x89, 0x16, 0x7f, 
	0xc3, 0xcb, 0x2f, 0x57, 0x90, 0x4, 0xfa, 0xec, 0x2e, 0xf, 
	0x93, 0xf0, 0xe7, 0x5f, 0x43, 0x97, 0x67, 0x87, 0xf1, 0x7b, 
	0x76, 0x7a, 0x60, 0x80, 0x8c, 0x93, 0x5b, 0xaa, 0x9f, 0x86, 
	0x47, 0x8b, 0x39, 0x62, 0x90, 0xeb, 0xd7, 0x19, 0x10, 0x73, 
	0x86, 0xe1, 0x1a, 0xda, 0xfe, 0x8e, 0x96, 0xc3, 0xe3, 0x74, 
	0x1d, 0x31, 0xa4, 0xbb, 0xd7, 0x19, 0x10, 0xd3, 0x64, 0x38, 
	0x90, 0xb4, 0x3c, 0xeb, 0xcd, 0x62, 0x2d, 0x4c, 0x9d, 0xfa, 
	0xf7, 0x79, 0x9e, 0x95, 0xb5, 0xa4, 0x30, 0xd7, 0xa8, 0xaa, 
	0x1, 0xca, 0x6a, 0xa, 0xb2, 0x4, 0xd, 0xa2, 0x26, 0xe1, 
	0x1b, 0xfe, 0x34, 0xc1, 0x11, 0xd4, 0xd2, 0x34, 0xf8, 0x17, 
	0x9a, 0x49, 0x15, 0x3f, 0xac, 0xc, 00, 00, };

static const char data_devconf_js[] PROGMEM = {
	/* /devconf.js */
	0x2f, 0x64, 0x65, 0x76, 0x63, 0x6f, 0x6e, 0x66, 0x2e, 0x6a, 0x73, 0,
	0x3c, 0x73, 0x63, 0x72, 0x69, 0x70, 0x74, 0x20, 0x74, 0x79, 
	0x70, 0x65, 0x3d, 0x22, 0x74, 0x65, 0x78, 0x74, 0x2f, 0x6a, 
	0x61, 0x76, 0x61, 0x73, 0x63, 0x72, 0x69, 0x70, 0x74, 0x22, 
	0x3e, 0xa, 0x20, 0x20, 0x66, 0x75, 0x6e, 0x63, 0x74, 0x69, 
	0x6f, 0x6e, 0x20, 0x64, 0x6f, 0x43, 0x6f, 0x6e, 0x66, 0x69, 
	0x67, 0x28, 0x29, 0x20, 0x7b, 0xa, 0x20, 0x20, 0x20, 0x20, 
	0x64, 0x6f, 0x4e, 0x65, 0x74, 0x53, 0x65, 0x74, 0x28, 0x30, 
	0x2c, 0x22, 0x31, 0x39, 0x32, 0x2e, 0x31, 0x36, 0x38, 0x2e, 
	0x32, 0x2e, 0x38, 0x38, 0x22, 0x2c, 0x20, 0x22, 0x32, 0x35, 
	0x35, 0x2e, 0x32, 0x35, 0x35, 0x2e, 0x32, 0x35, 0x35, 0x2e, 
	0x30, 0x22, 0x2c, 0x20, 0x22, 0x31, 0x39, 0x32, 0x2e, 0x31, 
	0x36, 0x38, 0x2e, 0x32, 0x2e, 0x31, 0x22, 0x29, 0x3b, 0xa, 
	0x20, 0x20, 0x20, 0x20, 0x2f, 0x2f, 0x64, 0x6f, 0x4e, 0x54, 
	0x50, 0x73, 0x65, 0x74, 0x28, 0x29, 0x3b, 0xa, 0x20, 0x20, 
	0x20, 0x20, 0x2f, 0x2f, 0x64, 0x6f, 0x55, 0x73, 0x65, 0x72, 
	0x53, 0x65, 0x74, 0x28, 0x29, 0x3b, 0xa, 0x20, 0x20, 0x7d, 
	0xa, 0x3c, 0x2f, 0x73, 0x63, 0x72, 0x69, 0x70, 0x74, 0x3e, 
	0xa, };

static const char data_404_html[] PROGMEM = {
	/* /404.html */
	0x2f, 0x34, 0x30, 0x34, 0x2e, 0x68, 0x74, 0x6d, 0x6c, 0,
	0x3c, 0x68, 0x74, 0x6d, 0x6c, 0x3e, 0xa, 0x20, 0x20, 0x3c, 
	0x62, 0x6f, 0x64, 0x79, 0x20, 0x62, 0x67, 0x63, 0x6f, 0x6c, 
	0x6f, 0x72, 0x3d, 0x22, 0x77, 0x68, 0x69, 0x74, 0x65, 0x22, 
	0x3e, 0xa, 0x20, 0x20, 0x20, 0x20, 0x3c, 0x63, 0x65, 0x6e, 
	0x74, 0x65, 0x72, 0x3e, 0xa, 0x20, 0x20, 0x20, 0x20, 0x20, 
	0x20, 0x3c, 0x68, 0x31, 0x3e, 0x34, 0x30, 0x34, 0x20, 0x2d, 
	0x20, 0x66, 0x69, 0x6c, 0x65, 0x20, 0x6e, 0x6f, 0x74, 0x20, 
	0x66, 0x6f, 0x75, 0x6e, 0x64, 0x3c, 0x2f, 0x68, 0x31, 0x3e, 
	0xa, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3c, 0x68, 0x33, 
	0x3e, 0x47, 0x6f, 0x20, 0x3c, 0x61, 0x20, 0x68, 0x72, 0x65, 
	0x66, 0x3d, 0x22, 0x2f, 0x22, 0x3e, 0x68, 0x65, 0x72, 0x65, 
	0x3c, 0x2f, 0x61, 0x3e, 0x20, 0x69, 0x6e, 0x73, 0x74, 0x65, 
	0x61, 0x64, 0x2e, 0x3c, 0x2f, 0x68, 0x33, 0x3e, 0xa, 0x20, 
	0x20, 0x20, 0x20, 0x3c, 0x2f, 0x63, 0x65, 0x6e, 0x74, 0x65, 
	0x72, 0x3e, 0xa, 0x20, 0x20, 0x3c, 0x2f, 0x62, 0x6f, 0x64, 
	0x79, 0x3e, 0xa, 0x3c, 0x2f, 0x68, 0x74, 0x6d, 0x6c, 0x3e, 
};

static const char data_footer_html[] PROGMEM = {
	/* /footer.html */
	0x2f, 0x66, 0x6f, 0x6f, 0x74, 0x65, 0x72, 0x2e, 0x68, 0x74, 0x6d, 0x6c, 0,
	0x20, 0x20, 0x3c, 0x2f, 0x62, 0x6f, 0x64, 0x79, 0x3e, 0xa, 
	0x3c, 0x2f, 0x68, 0x74, 0x6d, 0x6c, 0x3e, };

static const char data_menu_html[] PROGMEM = {
	/* /menu.html */
	0x2f, 0x6d, 0x65, 0x6e, 0x75, 0x2e, 0x68, 0x74, 0x6d, 0x6c, 0,
	0x20, 0x20, 0x3c, 0x62, 0x6f, 0x64, 0x79, 0x20, 0x62, 0x67, 
	0x63, 0x6f, 0x6c, 0x6f, 0x72, 0x3d, 0x22, 0x23, 0x66, 0x66, 
	0x66, 0x65, 0x65, 0x63, 0x22, 0x20, 0x74, 0x65, 0x78, 0x74, 
	0x3d, 0x22, 0x62, 0x6c, 0x61, 0x63, 0x6b, 0x22, 0x3e, 0xa, 
	0x20, 0x20, 0x3c, 0x64, 0x69, 0x76, 0x20, 0x63, 0x6c, 0x61, 
	0x73, 0x73, 0x3d, 0x22, 0x6d, 0x65, 0x6e, 0x75, 0x22, 0x3e, 
	0xa, 0x20, 0x20, 0x3c, 0x64, 0x69, 0x76, 0x20, 0x63, 0x6c, 
	0x61, 0x73, 0x73, 0x3d, 0x22, 0x6d, 0x65, 0x6e, 0x75, 0x62, 
	0x6f, 0x78, 0x22, 0x3e, 0x3c, 0x61, 0x20, 0x68, 0x72, 0x65, 
	0x66, 0x3d, 0x22, 0x2f, 0x22, 0x3e, 0x46, 0x72, 0x6f, 0x6e, 
	0x74, 0x20, 0x70, 0x61, 0x67, 0x65, 0x3c, 0x2f, 0x61, 0x3e, 
	0x3c, 0x2f, 0x64, 0x69, 0x76, 0x3e, 0xa, 0x20, 0x20, 0x3c, 
	0x64, 0x69, 0x76, 0x20, 0x63, 0x6c, 0x61, 0x73, 0x73, 0x3d, 
	0x22, 0x6d, 0x65, 0x6e, 0x75, 0x62, 0x6f, 0x78, 0x22, 0x3e, 
	0x3c, 0x61, 0x20, 0x68, 0x72, 0x65, 0x66, 0x3d, 0x22, 0x66, 
	0x69, 0x6c, 0x65, 0x73, 0x2e, 0x73, 0x68, 0x74, 0x6d, 0x6c, 
	0x22, 0x3e, 0x46, 0x69, 0x6c, 0x65, 0x20, 0x73, 0x74, 0x61, 
	0x74, 0x69, 0x73, 0x74, 0x69, 0x63, 0x73, 0x3c, 0x2f, 0x61, 
	0x3e, 0x3c, 0x2f, 0x64, 0x69, 0x76, 0x3e, 0xa, 0x20, 0x20, 
	0x3c, 0x64, 0x69, 0x76, 0x20, 0x63, 0x6c, 0x61, 0x73, 0x73, 
	0x3d, 0x22, 0x6d, 0x65, 0x6e, 0x75, 0x62, 0x6f, 0x78, 0x22, 
	0x3e, 0x3c, 0x61, 0x20, 0x68, 0x72, 0x65, 0x66, 0x3d, 0x22, 
	0x73, 0x74, 0x61, 0x74, 0x73, 0x2e, 0x73, 0x68, 0x74, 0x6d, 
	0x6c, 0x22, 0x3e, 0x4e, 0x65, 0x74, 0x77, 0x6f, 0x72, 0x6b, 
	0x20, 0x73, 0x74, 0x61, 0x74, 0x69, 0x73, 0x74, 0x69, 0x63, 
	0x73, 0x3c, 0x2f, 0x61, 0x3e, 0x3c, 0x2f, 0x64, 0x69, 0x76, 
	0x3e, 0xa, 0x20, 0x20, 0x3c, 0x64, 0x69, 0x76, 0x20, 0x63, 
	0x6c, 0x61, 0x73, 0x73, 0x3d, 0x22, 0x6d, 0x65, 0x6e, 0x75, 
	0x62, 0x6f, 0x78, 0x22, 0x3e, 0x3c, 0x61, 0x20, 0x68, 0x72, 
	0x65, 0x66, 0x3d, 0x22, 0x74, 0x63, 0x70, 0x2e, 0x73, 0x68, 
	0x74, 0x6d, 0x6c, 0x22, 0x3e, 0x4e, 0x65, 0x74, 0x77, 0x6f, 
	0x72, 0x6b, 0xa, 0x20, 0x20, 0x63, 0x6f, 0x6e, 0x6e, 0x65, 
	0x63, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x3c, 0x2f, 0x61, 0x3e, 
	0x3c, 0x2f, 0x64, 0x69, 0x76, 0x3e, 0xa, 0x20, 0x20, 0x3c, 
	0x62, 0x72, 0x3e, 0xa, 0x20, 0x20, 0x3c, 0x2f, 0x64, 0x69, 
	0x76, 0x3e, 0xa, 0x20, 0x20, 0xa, 0x20, 0x20, 0x3c, 0x64, 
	0x69, 0x76, 0x20, 0x63, 0x6c, 0x61, 0x73, 0x73, 0x3d, 0x22, 
	0x63, 0x6f, 0x6e, 0x74, 0x65, 0x6e, 0x74, 0x62, 0x6c, 0x6f, 
	0x63, 0x6b, 0x22, 0x3e, 0xa, };

static const char data_style_css[] PROGMEM = {
	/* /style.css */
	0x2f, 0x73, 0x74, 0x79, 0x6c, 0x65, 0x2e, 0x63, 0x73, 0x73, 0,
	0x1f, 0x8b, 0x8, 0x8, 0xa0, 0x81, 0x6, 0x4c, 00, 0x3, 
	0x73, 0x74, 0x79, 0x6c, 0x65, 0x2e, 0x63, 0x73, 0x73, 0x5f, 
	0x67, 0x7a, 00, 0x95, 0x93, 0xcb, 0x6e, 0xc2, 0x30, 0x10, 
	0x45, 0xd7, 0xf5, 0x57, 0x58, 0xaa, 0xd8, 0x20, 0x2, 0x49, 
	0x44, 0xab, 0xe2, 0x7c, 0x8d, 0x63, 0x4f, 0x12, 0xb, 0xc7, 
	0x13, 0x19, 0xf3, 0x2a, 0xea, 0xbf, 0xd7, 0xf, 0x8a, 0x2, 
	0x8d, 0x54, 0xba, 0xcc, 0x75, 0x7c, 0xe7, 0xcc, 0x9d, 0x71, 
	0x57, 0x50, 0x72, 0x21, 0x94, 0x3a, 0x38, 0xb9, 0x8c, 0x6b, 
	0xd5, 0x1a, 0x46, 0x5, 0x18, 0x7, 0xb6, 0xf2, 0x6a, 0x83, 
	0xc6, 0x65, 0x3b, 0xf5, 0x9, 0xac, 0x58, 0xf, 0xee, 0xa6, 
	0x34, 0xbc, 0x57, 0xfa, 0xcc, 0xb8, 0x55, 0x5c, 0x2f, 0x3a, 
	0xd0, 0x7, 0x70, 0x4a, 0xf0, 0xdb, 0xf1, 0x11, 0x54, 0xdb, 
	0x39, 0x56, 0xa3, 0x96, 0x41, 0x1b, 0xb8, 0x94, 0xca, 0xb4, 
	0xac, 0xc8, 0x87, 0x53, 0x45, 0xc9, 0x17, 0x21, 0x35, 0xca, 
	0xb3, 0xaf, 0xea, 0xcf, 0x6a, 0x2e, 0xb6, 0xad, 0xc5, 0xbd, 
	0x91, 0x99, 0x40, 0x8d, 0x96, 0xd1, 0xd7, 0xa6, 0x69, 00, 
	0x44, 0xb8, 0x98, 0x94, 0x5a, 0xfb, 0x7f, 0x2a, 0x72, 0x47, 
	0xf3, 0xf1, 0x4, 0x8c, 0xaf, 0xb3, 0xec, 0xc1, 0xec, 0x63, 
	0x7b, 0x3d, 0xb7, 0xad, 0xf2, 0xad, 0xad, 0x3d, 0x82, 0xff, 
	0x3c, 0x2a, 0xe9, 0x3a, 0xf6, 0x9e, 0xcf, 0xa2, 0xef, 0xf, 
	0x60, 0x19, 0xe, 0x5f, 0x2, 0x15, 0x5a, 0x9, 0x9e, 0x65, 
	0x87, 0x5a, 0x49, 0x5a, 0xa4, 0x3b, 0xd3, 0xa8, 0x42, 0x96, 
	0xd5, 0x7d, 0x7c, 0x1a, 0x9a, 0x8, 0x77, 0x7, 0xbc, 0xf9, 
	0x1b, 0x98, 0xc6, 0x68, 0xa4, 0x3a, 0x44, 0xea, 0x1a, 0x4f, 
	0x11, 0x3c, 0x91, 0xd2, 0xf2, 0x6d, 0x56, 0x8d, 0xc0, 0xf2, 
	0x68, 0xa6, 0x91, 0x3b, 0x46, 0x53, 0xbd, 0xa9, 0xf9, 0x85, 
	0x4, 0x84, 0xaf, 0xe8, 0x3f, 0x6b, 0x8d, 0x62, 0x4b, 0x2e, 
	0x91, 0xea, 0xf9, 0x2c, 0x46, 0x15, 0x7d, 0x8, 0x54, 0xa2, 
	0x73, 0x20, 0xa7, 0xb3, 0x38, 0x76, 0xca, 0xc1, 0xff, 0xa7, 
	0xe4, 0x81, 0x2, 0xe6, 0xb0, 0x54, 0xc6, 0x59, 0x1c, 0x8d, 
	0x2a, 0xb, 0x6d, 0xb1, 0x32, 0x4f, 0x8c, 0x57, 0xcd, 0xc6, 
	0xbd, 0x4a, 0xe2, 0xfd, 0x76, 0xe6, 0xa1, 0xd4, 0x6a, 0x3e, 
	0xb1, 0x80, 0x74, 0xbe, 0x7a, 0x2a, 0xf8, 0x61, 0x29, 0xb4, 
	0x32, 0xdb, 0x88, 0x30, 0x32, 0x2e, 0x7f, 0xf7, 0x20, 0x70, 
	0x6f, 0x15, 0xd8, 0x45, 0x8f, 0x6, 0x77, 0x3, 0x17, 0x50, 
	0xc5, 0x58, 0x47, 0x13, 0x18, 0xd, 0xe0, 0x6a, 0xbb, 0x79, 
	0xf0, 0x9d, 0xd8, 0x87, 0x7f, 0xd9, 0x92, 0x21, 0x1a, 0x5e, 
	0xa7, 0x95, 0xc2, 0x8a, 0xef, 0x2b, 0xd5, 0x8c, 0x41, 0x3d, 
	0xbe, 0xeb, 0x28, 0xa6, 0x7, 0xf8, 0xd, 0xe4, 0x36, 0x79, 
	0x10, 0xf6, 0x3, 00, 00, };

static const char data_fade_png[] PROGMEM = {
	/* /fade.png */
	0x2f, 0x66, 0x61, 0x64, 0x65, 0x2e, 0x70, 0x6e, 0x67, 0,
	0x89, 0x50, 0x4e, 0x47, 0xd, 0xa, 0x1a, 0xa, 00, 00, 
	00, 0xd, 0x49, 0x48, 0x44, 0x52, 00, 00, 00, 0x4, 
	00, 00, 00, 0xa, 0x8, 0x2, 00, 00, 00, 0x1c, 
	0x99, 0x68, 0x59, 00, 00, 00, 0x9, 0x70, 0x48, 0x59, 
	0x73, 00, 00, 0xb, 0x13, 00, 00, 0xb, 0x13, 0x1, 
	00, 0x9a, 0x9c, 0x18, 00, 00, 00, 0x7, 0x74, 0x49, 
	0x4d, 0x45, 0x7, 0xd6, 0x6, 0x8, 0x14, 0x1b, 0x39, 0xaf, 
	0x5b, 0xc0, 0xe3, 00, 00, 00, 0x1d, 0x74, 0x45, 0x58, 
	0x74, 0x43, 0x6f, 0x6d, 0x6d, 0x65, 0x6e, 0x74, 00, 0x43, 
	0x72, 0x65, 0x61, 0x74, 0x65, 0x64, 0x20, 0x77, 0x69, 0x74, 
	0x68, 0x20, 0x54, 0x68, 0x65, 0x20, 0x47, 0x49, 0x4d, 0x50, 
	0xef, 0x64, 0x25, 0x6e, 00, 00, 00, 0x3a, 0x49, 0x44, 
	0x41, 0x54, 0x8, 0xd7, 0x75, 0x8c, 0x31, 0x12, 00, 0x10, 
	0x10, 0xc4, 0x2e, 0x37, 0x9e, 0x40, 0x65, 0xfd, 0xff, 0x83, 
	0xf4, 0xa, 0x1c, 0x8d, 0x54, 0x9b, 0xc9, 0xcc, 0x9a, 0x3d, 
	0x90, 0x73, 0x71, 0x67, 0x91, 0xd4, 0x74, 0x36, 0xa9, 0x55, 
	0x1, 0xf8, 0x29, 0x58, 0xc8, 0xbf, 0x48, 0xc4, 0x81, 0x74, 
	0xb, 0xa3, 0xf, 0x7c, 0xdb, 0x4, 0xe8, 0x40, 0x5, 0xdf, 
	0xa1, 0xf3, 0xfc, 0x73, 00, 00, 00, 00, 0x49, 0x45, 
	0x4e, 0x44, 0xae, 0x42, 0x60, 0x82, };

static const char data_index_html[] PROGMEM = {
	/* /index.html */
	0x2f, 0x69, 0x6e, 0x64, 0x65, 0x78, 0x2e, 0x68, 0x74, 0x6d, 0x6c, 0,
	0x3c, 0x21, 0x44, 0x4f, 0x43, 0x54, 0x59, 0x50, 0x45, 0x20, 
	0x48, 0x54, 0x4d, 0x4c, 0x20, 0x50, 0x55, 0x42, 0x4c, 0x49, 
	0x43, 0x20, 0x22, 0x2d, 0x2f, 0x2f, 0x57, 0x33, 0x43, 0x2f, 
	0x2f, 0x44, 0x54, 0x44, 0x20, 0x48, 0x54, 0x4d, 0x4c, 0x20, 
	0x34, 0x2e, 0x30, 0x31, 0x20, 0x54, 0x72, 0x61, 0x6e, 0x73, 
	0x69, 0x74, 0x69, 0x6f, 0x6e, 0x61, 0x6c, 0x2f, 0x2f, 0x45, 
	0x4e, 0x22, 0x20, 0x22, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 
	0x2f, 0x77, 0x77, 0x77, 0x2e, 0x77, 0x33, 0x2e, 0x6f, 0x72, 
	0x67, 0x2f, 0x54, 0x52, 0x2f, 0x68, 0x74, 0x6d, 0x6c, 0x34, 
	0x2f, 0x6c, 0x6f, 0x6f, 0x73, 0x65, 0x2e, 0x64, 0x74, 0x64, 
	0x22, 0x3e, 0xa, 0x3c, 0x68, 0x74, 0x6d, 0x6c, 0x3e, 0xa, 
	0x20, 0x20, 0x3c, 0x68, 0x65, 0x61, 0x64, 0x3e, 0xa, 0x20, 
	0x20, 0x20, 0x20, 0x3c, 0x74, 0x69, 0x74, 0x6c, 0x65, 0x3e, 
	0x57, 0x65, 0x6c, 0x63, 0x6f, 0x6d, 0x65, 0x20, 0x74, 0x6f, 
	0x20, 0x74, 0x68, 0x65, 0x20, 0x75, 0x49, 0x50, 0x20, 0x77, 
	0x65, 0x62, 0x20, 0x73, 0x65, 0x72, 0x76, 0x65, 0x72, 0x21, 
	0x3c, 0x2f, 0x74, 0x69, 0x74, 0x6c, 0x65, 0x3e, 0xa, 0x20, 
	0x20, 0x20, 0x20, 0x3c, 0x6c, 0x69, 0x6e, 0x6b, 0x20, 0x72, 
	0x65, 0x6c, 0x3d, 0x22, 0x73, 0x74, 0x79, 0x6c, 0x65, 0x73, 
	0x68, 0x65, 0x65, 0x74, 0x22, 0x20, 0x74, 0x79, 0x70, 0x65, 
	0x3d, 0x22, 0x74, 0x65, 0x78, 0x74, 0x2f, 0x63, 0x73, 0x73, 
	0x22, 0x20, 0x68, 0x72, 0x65, 0x66, 0x3d, 0x22, 0x73, 0x74, 
	0x79, 0x6c, 0x65, 0x2e, 0x63, 0x73, 0x73, 0x22, 0x3e, 0x20, 
	0x20, 0xa, 0x20, 0x20, 0x3c, 0x2f, 0x68, 0x65, 0x61, 0x64, 
	0x3e, 0xa, 0x20, 0x20, 0x3c, 0x62, 0x6f, 0x64, 0x79, 0x20, 
	0x62, 0x67, 0x63, 0x6f, 0x6c, 0x6f, 0x72, 0x3d, 0x22, 0x23, 
	0x66, 0x66, 0x66, 0x65, 0x65, 0x63, 0x22, 0x20, 0x74, 0x65, 
	0x78, 0x74, 0x3d, 0x22, 0x62, 0x6c, 0x61, 0x63, 0x6b, 0x22, 
	0x3e, 0xa, 0xa, 0x20, 0x20, 0x3c, 0x64, 0x69, 0x76, 0x20, 
	0x63, 0x6c, 0x61, 0x73, 0x73, 0x3d, 0x22, 0x6d, 0x65, 0x6e, 
	0x75, 0x22, 0x3e, 0xa, 0x20, 0x20, 0x3c, 0x64, 0x69, 0x76, 
	0x20, 0x63, 0x6c, 0x61, 0x73, 0x73, 0x3d, 0x22, 0x6d, 0x65, 
	0x6e, 0x75, 0x62, 0x6f, 0x78, 0x22, 0x3e, 0x3c, 0x61, 0x20, 
	0x68, 0x72, 0x65, 0x66, 0x3d, 0x22, 0x2f, 0x22, 0x3e, 0x46, 
	0x72, 0x6f, 0x6e, 0x74, 0x20, 0x70, 0x61, 0x67, 0x65, 0x3c, 
	0x2f, 0x61, 0x3e, 0x3c, 0x2f, 0x64, 0x69, 0x76, 0x3e, 0xa, 
	0x20, 0x20, 0x3c, 0x64, 0x69, 0x76, 0x20, 0x63, 0x6c, 0x61, 
	0x73, 0x73, 0x3d, 0x22, 0x6d, 0x65, 0x6e, 0x75, 0x62, 0x6f, 
	0x78, 0x22, 0x3e, 0x3c, 0x61, 0x20, 0x68, 0x72, 0x65, 0x66, 
	0x3d, 0x22, 0x66, 0x69, 0x6c, 0x65, 0x73, 0x2e, 0x73, 0x68, 
	0x74, 0x6d, 0x6c, 0x22, 0x3e, 0x46, 0x69, 0x6c, 0x65, 0x20, 
	0x73, 0x74, 0x61, 0x74, 0x69, 0x73, 0x74, 0x69, 0x63, 0x73, 
	0x3c, 0x2f, 0x61, 0x3e, 0x3c, 0x2f, 0x64, 0x69, 0x76, 0x3e, 
	0xa, 0x20, 0x20, 0x3c, 0x64, 0x69, 0x76, 0x20, 0x63, 0x6c, 
	0x61, 0x73, 0x73, 0x3d, 0x22, 0x6d, 0x65, 0x6e, 0x75, 0x62, 
	0x6f, 0x78, 0x22, 0x3e, 0x3c, 0x61, 0x20, 0x68, 0x72, 0x65, 
	0x66, 0x3d, 0x22, 0x73, 0x74, 0x61, 0x74, 0x73, 0x2e, 0x73, 
	0x68, 0x74, 0x6d, 0x6c, 0x22, 0x3e, 0x4e, 0x65, 0x74, 0x77, 
	0x6f, 0x72, 0x6b, 0x20, 0x73, 0x74, 0x61, 0x74, 0x69, 0x73, 
	0x74, 0x69, 0x63, 0x73, 0x3c, 0x2f, 0x61, 0x3e, 0x3c, 0x2f, 
	0x64, 0x69, 0x76, 0x3e, 0xa, 0x20, 0x20, 0x3c, 0x64, 0x69, 
	0x76, 0x20, 0x63, 0x6c, 0x61, 0x73, 0x73, 0x3d, 0x22, 0x6d, 
	0x65, 0x6e, 0x75, 0x62, 0x6f, 0x78, 0x22, 0x3e, 0x3c, 0x61, 
	0x20, 0x68, 0x72, 0x65, 0x66, 0x3d, 0x22, 0x74, 0x63, 0x70, 
	0x2e, 0x73, 0x68, 0x74, 0x6d, 0x6c, 0x22, 0x3e, 0x4e, 0x65, 
	0x74, 0x77, 0x6f, 0x72, 0x6b, 0xa, 0x20, 0x20, 0x63, 0x6f, 
	0x6e, 0x6e, 0x65, 0x63, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x3c, 
	0x2f, 0x61, 0x3e, 0x3c, 0x2f, 0x64, 0x69, 0x76, 0x3e, 0xa, 
	0x20, 0x20, 0x3c, 0x62, 0x72, 0x3e, 0xa, 0x20, 0x20, 0x3c, 
	0x2f, 0x64, 0x69, 0x76, 0x3e, 0xa, 0xa, 0x20, 0x20, 0x3c, 
	0x64, 0x69, 0x76, 0x20, 0x63, 0x6c, 0x61, 0x73, 0x73, 0x3d, 
	0x22, 0x63, 0x6f, 0x6e, 0x74, 0x65, 0x6e, 0x74, 0x62, 0x6c, 
	0x6f, 0x63, 0x6b, 0x22, 0x3e, 0xa, 0x20, 0x20, 0x3c, 0x70, 
	0x3e, 0xa, 0x20, 0x20, 0x54, 0x68, 0x65, 0x73, 0x65, 0x20, 
	0x77, 0x65, 0x62, 0x20, 0x70, 0x61, 0x67, 0x65, 0x73, 0x20, 
	0x61, 0x72, 0x65, 0x20, 0x73, 0x65, 0x72, 0x76, 0x65, 0x64, 
	0x20, 0x62, 0x79, 0x20, 0x61, 0x20, 0x73, 0x6d, 0x61, 0x6c, 
	0x6c, 0x20, 0x77, 0x65, 0x62, 0x20, 0x73, 0x65, 0x72, 0x76, 
	0x65, 0x72, 0x20, 0x72, 0x75, 0x6e, 0x6e, 0x69, 0x6e, 0x67, 
	0x20, 0x6f, 0x6e, 0x20, 0x74, 0x6f, 0x70, 0x20, 0x6f, 0x66, 
	0xa, 0x20, 0x20, 0x74, 0x68, 0x65, 0x20, 0x3c, 0x61, 0x20, 
	0x68, 0x72, 0x65, 0x66, 0x3d, 0x22, 0x68, 0x74, 0x74, 0x70, 
	0x3a, 0x2f, 0x2f, 0x77, 0x77, 0x77, 0x2e, 0x73, 0x69, 0x63, 
	0x73, 0x2e, 0x73, 0x65, 0x2f, 0x7e, 0x61, 0x64, 0x61, 0x6d, 
	0x2f, 0x75, 0x69, 0x70, 0x2f, 0x22, 0x3e, 0x75, 0x49, 0x50, 
	0x20, 0x65, 0x6d, 0x62, 0x65, 0x64, 0x64, 0x65, 0x64, 0x20, 
	0x54, 0x43, 0x50, 0x2f, 0x49, 0x50, 0xa, 0x20, 0x20, 0x73, 
	0x74, 0x61, 0x63, 0x6b, 0x3c, 0x2f, 0x61, 0x3e, 0x2e, 0xa, 
	0x20, 0x20, 0x3c, 0x2f, 0x70, 0x3e, 0xa, 0x20, 0x20, 0x3c, 
	0x70, 0x3e, 0xa, 0x20, 0x20, 0x43, 0x6c, 0x69, 0x63, 0x6b, 
	0x20, 0x6f, 0x6e, 0x20, 0x74, 0x68, 0x65, 0x20, 0x6c, 0x69, 
	0x6e, 0x6b, 0x73, 0x20, 0x61, 0x62, 0x6f, 0x76, 0x65, 0x20, 
	0x66, 0x6f, 0x72, 0x20, 0x77, 0x65, 0x62, 0x20, 0x73, 0x65, 
	0x72, 0x76, 0x65, 0x72, 0x20, 0x73, 0x74, 0x61, 0x74, 0x69, 
	0x73, 0x74, 0x69, 0x63, 0x73, 0x2e, 0xa, 0x20, 0x20, 0x3c, 
	0x2f, 0x70, 0x3e, 0xa, 0xa, 0x20, 0x20, 0x3c, 0x2f, 0x62, 
	0x6f, 0x64, 0x79, 0x3e, 0xa, 0x3c, 0x2f, 0x68, 0x74, 0x6d, 
	0x6c, 0x3e, 0xa, };

const struct httpd_fsdata_file file_header_html[] = {{NULL, data_header_html, data_header_html + 13, (int) sizeof(data_header_html) - 13, 0}};

const struct httpd_fsdata_file file_devconf_html[] = {{file_header_html, data_devconf_html, data_devconf_html + 14, (int) sizeof(data_devconf_html) - 14, 1}};

const struct httpd_fsdata_file file_devconf_js[] = {{file_devconf_html, data_devconf_js, data_devconf_js + 12, (int) sizeof(data_devconf_js) - 12, 0}};

const struct httpd_fsdata_file file_404_html[] = {{file_devconf_js, data_404_html, data_404_html + 10, (int) sizeof(data_404_html) - 10, 0}};

const struct httpd_fsdata_file file_footer_html[] = {{file_404_html, data_footer_html, data_footer_html + 13, (int) sizeof(data_footer_html) - 13, 0}};

const struct httpd_fsdata_file file_menu_html[] = {{file_footer_html, data_menu_html, data_menu_html + 11, (int) sizeof(data_menu_html) - 11, 0}};

const struct httpd_fsdata_file file_style_css[] = {{file_menu_html, data_style_css, data_style_css + 11, (int) sizeof(data_style_css) - 11, 1}};

const struct httpd_fsdata_file file_fade_png[] = {{file_style_css, data_fade_png, data_fade_png + 10, (int) sizeof(data_fade_png) - 10, 0}};

const struct httpd_fsdata_file file_index_html[] = {{file_fade_png, data_index_html, data_index_html + 12, (int) sizeof(data_index_html) - 12, 0}};

#define HTTPD_FS_ROOT file_index_html

#define HTTPD_FS_NUMFILES 9
