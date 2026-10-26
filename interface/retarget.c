// Arm 标准库非半主机接口；GCC 由 nosys.specs 提供系统调用。
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)

#include <stddef.h>
#include <rt_sys.h>

__asm(".global __use_no_semihosting\n");

const char __stdin_name[] = ":tt";
const char __stdout_name[] = ":tt";
const char __stderr_name[] = ":tt";

FILEHANDLE _sys_open(const char* name, int openmode) {
  (void)name;
  (void)openmode;
  return 1;
}

int _sys_close(FILEHANDLE fh) {
  (void)fh;
  return 0;
}

char* _sys_command_string(char* cmd, int len) {
  (void)cmd;
  (void)len;
  return NULL;
}

// 沿用轮腿工程：接收并丢弃输出，暂不绑定串口。
int _sys_write(FILEHANDLE fh, const unsigned char* buf, unsigned len, int mode) {
  (void)fh;
  (void)buf;
  (void)len;
  (void)mode;
  return 0;
}

int _sys_read(FILEHANDLE fh, unsigned char* buf, unsigned len, int mode) {
  (void)fh;
  (void)buf;
  (void)len;
  (void)mode;
  return -1;
}

void _ttywrch(int ch) {
  (void)ch;
}

int _sys_istty(FILEHANDLE fh) {
  (void)fh;
  return 0;
}

int _sys_seek(FILEHANDLE fh, long pos) {
  (void)fh;
  (void)pos;
  return -1;
}

long _sys_flen(FILEHANDLE fh) {
  (void)fh;
  return -1;
}

void _sys_exit(int return_code) {
  (void)return_code;
  for (;;) {}
}

#endif
