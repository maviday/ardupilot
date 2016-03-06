"""
WAF Tool to select the correct toolchain based on the target archtecture.

This tool must be loaded before compiler tools. Use the environment variable
TOOLCHAIN to define the toolchain prefix.

Example::

    def configure(cfg):
        cfg.env.TOOLCHAIN = 'arm-linux-gnueabihf'
        cfg.load('toolchain')
        cfg.load('cxx_compiler')
"""

from waflib import Utils
import os

suffixes = dict(
    CXX='g++',
    CC='gcc',
    AS='gcc',
    AR='ar',
    LD='g++',
    GDB='gdb',
    OBJCOPY='objcopy',
)

def configure(cfg):
    if not cfg.env.TOOLCHAIN:
        cfg.env.TOOLCHAIN = 'native'
        prefix = ''
    else:
        cfg.env.TOOLCHAIN = Utils.to_list(cfg.env.TOOLCHAIN)[0]
        cfg.msg('Using toolchain prefix', cfg.env.TOOLCHAIN)
        prefix = cfg.env.TOOLCHAIN + '-'

    compiler = cfg.environ.get('CC') or cfg.options.check_c_compiler
    if 'clang' in compiler:
        if cfg.env.TOOLCHAIN != 'native':
            cfg.msg('CC was: ', cfg.environ['CC'])
            cfg.msg('CXX was: ', cfg.environ['CXX'])
            cfg.environ['CC'] = compiler + ' -v --target=' + cfg.env.TOOLCHAIN + ' gcc-toolchain=/home/travis/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/'
            cfg.environ['CXX'] = compiler + '++ -v --target=' + cfg.env.TOOLCHAIN + ' gcc-toolchain=/home/travis/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/'
            cfg.msg('CC is: ', cfg.environ['CC'])
            cfg.msg('CXX is: ', cfg.environ['CXX'])
            # cfg.env.CXXFLAGS += [
                # '-v',
                # '--target=' + cfg.env.TOOLCHAIN,
                # '--sysroot=/home/travis/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/' + cfg.env.TOOLCHAIN + '/libc',
                # '-B/home/travis/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/'
                # # '-I/home/travis/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/arm-linux-gnueabihf/include/c++/4.8.3/tr1',
                # # '-I/home/travis/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/arm-linux-gnueabihf/include/c++/4.8.3/parallel',
                # # '-I/home/ubuntu/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/arm-linux-gnueabihf/include/c++/4.8.3/tr2',
                # # '-I/home/ubuntu/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/arm-linux-gnueabihf/include/c++/4.8.3/bits',
                # # '-I/home/ubuntu/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/arm-linux-gnueabihf/include/c++/4.8.3/debug',
                # # '-I/home/ubuntu/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/arm-linux-gnueabihf/include/c++/4.8.3/decimal',
                # # '-I/home/ubuntu/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/arm-linux-gnueabihf/include/c++/4.8.3/ext',
                # # '-I/home/ubuntu/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/arm-linux-gnueabihf/include/c++/4.8.3/profile'
            # ]
            # cfg.env.LINKFLAGS += [
                # '-v',
                # '--target=' + cfg.env.TOOLCHAIN,
                # '--sysroot=/home/travis/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/' + cfg.env.TOOLCHAIN + '/libc',
                # '-B/home/travis/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/'
            # ]
    else:
        for k in suffixes:
            cfg.env.append_value(k, prefix + suffixes[k])
        cfg.environ.pop('CC', None)
        cfg.environ.pop('CXX', None)
