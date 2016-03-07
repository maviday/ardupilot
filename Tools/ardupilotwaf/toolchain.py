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


    if cfg.env.TOOLCHAIN != 'native':
        for k in suffixes:
            cfg.env.append_value(k, prefix + suffixes[k])
        
        c_compiler = cfg.options.check_c_compiler or cfg.environ.get('CC')
        cxx_compiler = cfg.options.check_cxx_compiler or cfg.environ.get('CXX')
        cfg.environ.pop('CC', None)
        cfg.environ.pop('CXX', None)

        if 'clang' in c_compiler:
            toolchain_path = os.path.abspath(os.path.join(os.path.dirname(cfg.find_file(cfg.env['AR'], cfg.environ.get('PATH', '').split(os.pathsep))), '..'))
            
            cfg.env['CC'] = c_compiler
            cfg.env.CFLAGS += [
                '-v',
                '--target=' + cfg.env.TOOLCHAIN,
                '--gcc-toolchain=' + toolchain_path,
                '--sysroot=' + os.path.join(toolchain_path, cfg.env.TOOLCHAIN, 'libc'),
                '-B' + os.path.join(toolchain_path, 'bin')
            ]
            cfg.env.LINKFLAGS += [
                '-v',
                '--target=' + cfg.env.TOOLCHAIN,
                '--gcc-toolchain=' + toolchain_path,
                '--sysroot=' + os.path.join(toolchain_path, cfg.env.TOOLCHAIN, 'libc'),
                '-B' + os.path.join(toolchain_path, 'bin')
            ]

        if 'clang++' in cxx_compiler:
            toolchain_path = os.path.abspath(os.path.join(os.path.dirname(cfg.find_file(cfg.env['AR'], cfg.environ.get('PATH', '').split(os.pathsep))), '..'))

            cfg.env['CXX'] = cxx_compiler
            cfg.env.CXXFLAGS += [
                '-v',
                '--target=' + cfg.env.TOOLCHAIN,
                '--gcc-toolchain=' + toolchain_path,
                '--sysroot=' + os.path.join(toolchain_path, cfg.env.TOOLCHAIN, 'libc'),
                '-B' + os.path.join(toolchain_path, 'bin')
            ]
            cfg.env.LINKFLAGS += [
                '-v',
                '--target=' + cfg.env.TOOLCHAIN,
                '--gcc-toolchain=' + toolchain_path,
                '--sysroot=' + os.path.join(toolchain_path, cfg.env.TOOLCHAIN, 'libc'),
                '-B' + os.path.join(toolchain_path, 'bin')
            ]