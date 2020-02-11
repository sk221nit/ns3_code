# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

# def options(opt):
#     pass

# def configure(conf):
#     conf.check_nonfatal(header_name='stdint.h', define_name='HAVE_STDINT_H')

def build(bld):
    module = bld.create_ns3_module('wave', ['core','wifi', 'propagation', 'internet'])
    module.source = [
        'model/wave-mac-low.cc',
        'model/ocb-wifi-mac.cc',
        'model/vendor-specific-action.cc',
        'model/channel-coordinator.cc',
        'model/channel-scheduler.cc',
        'model/default-channel-scheduler.cc',
        'model/channel-manager.cc',
        'model/vsa-manager.cc',
        'model/bsm-application.cc',
        'model/bsm-header.cc',
        'model/neighbor-table.cc',
        'model/neighbor-table-header.cc',
        'model/higher-tx-tag.cc',
        'model/wave-net-device.cc',
        'helper/wave-bsm-stats.cc',
        'helper/wave-mac-helper.cc',
        'helper/wave-helper.cc',
        'helper/wifi-80211p-helper.cc',
        'helper/wave-bsm-helper.cc'
        ]

    module_test = bld.create_ns3_module_test_library('wave')
    module_test.source = [
        'test/mac-extension-test-suite.cc',
        'test/ocb-test-suite.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'wave'
    headers.source = [
        'model/wave-mac-low.h',
        'model/ocb-wifi-mac.h',
        'model/vendor-specific-action.h',
        'model/channel-coordinator.h',
        'model/channel-manager.h',
        'model/channel-scheduler.h',
        'model/default-channel-scheduler.h',
        'model/vsa-manager.h',
        'model/higher-tx-tag.h',
        'model/wave-net-device.h',
        'model/bsm-application.h',
        'model/bsm-header.h',
        'model/neighbor-table.h',
        'model/neighbor-table-header.h',
        'helper/wave-bsm-stats.h',
        'helper/wave-mac-helper.h',
        'helper/wave-helper.h',
        'helper/wifi-80211p-helper.h',
        'helper/wave-bsm-helper.h',
        ]

    if bld.env.ENABLE_EXAMPLES:
        bld.recurse('examples')

    bld.ns3_python_bindings()

