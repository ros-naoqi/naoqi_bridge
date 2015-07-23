#!/usr/bin/env python

#
# Copyright 2014 Aldebaran Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Useful constants used by the Vision modules.

# Camera model
kOV7670  = 1            # VGA camera
kMT9M114 = 2            # HD wide angle camera

# Image format
k960p = 3                # 1280*960
k4VGA = 3                # 1280*960
kVGA = 2                 # 640*480
kQVGA = 1                # 320*240
kQQVGA = 0               # 160*120

# Color Space
kYuvColorSpace = 0
kyUvColorSpace = 1
kyuVColorSpace = 2
kRgbColorSpace = 3
krGbColorSpace = 4
krgBColorSpace = 5
kHsvColorSpace = 6
khSvColorSpace = 7
khsVColorSpace = 8
kYUV422InterlacedColorSpace = 9  #deprecated
kYUV422ColorSpace = 9
kYUVColorSpace = 10
kRGBColorSpace = 11
kHSVColorSpace = 12
kBGRColorSpace = 13
kYYCbCrColorSpace = 14
kH2RGBColorSpace = 15
kHSMixedColorSpace = 16
kDepthColorSpace = 17
kRawDepthColorSpace = 23

# Scale methods
kSimpleScaleMethod = 0
kAverageScaleMethod = 1
kQualityScaleMethod = 2
kNoScaling = 3


# Standard Id
kCameraBrightnessID       = 0
kCameraContrastID         = 1
kCameraSaturationID       = 2
kCameraHueID              = 3
kCameraRedChromaID        = 4
kCameraBlueChromaID       = 5
kCameraGainID             = 6
kCameraHFlipID            = 7
kCameraVFlipID            = 8
kCameraLensXID            = 9
kCameraLensYID            = 10
kCameraAutoExpositionID   = 11
kCameraAutoWhiteBalanceID = 12
kCameraAutoGainID         = 13
kCameraResolutionID       = 14
kCameraFrameRateID        = 15
kCameraBufferSizeID       = 16
kCameraExposureID         = 17
kCameraSelectID           = 18
kCameraSetDefaultParamsID = 19
kCameraColorSpaceID       = 20
kCameraExposureCorrectionID = 21
kCameraAecAlgorithmID     = 22
kCameraFastSwitchID       = 23
kCameraSharpnessID        = 24
kCameraAwbGreenGainID     = 25
kCameraAblcID             = 26
kCameraAblcTargetID       = 27
kCameraAblcStableRangeID  = 28
kCameraBlcBlueID          = 29
kCameraBlcRedID           = 30
kCameraBlcGbID            = 31
kCameraBlcGrID            = 32
kCameraWhiteBalanceID     = 33
kCameraBacklightCompensationID = 34
