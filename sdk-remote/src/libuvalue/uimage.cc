/*
 * Copyright (C) 2009-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

/// \file libuvalue/uimage.cc

#include <libport/debug.hh>
#include <libport/format.hh>
#include <urbi/uimage.hh>

#define cardinality_of(Array) (sizeof (Array) / sizeof (*(Array)))

GD_CATEGORY(UValue);

namespace urbi
{

  /*---------------.
  | UImageFormat.  |
  `---------------*/

  static const char* formats[] =
  {
    "image_unknown",
    "rgb",
    "YCbCr",
    "jpeg",
    "ppm",
    "YUV422",
    "grey8",
    "grey4",
      "yuv411_planar",
  };

  const char*
  format_string(UImageFormat f)
  {
    if (f < 0 || int(cardinality_of(formats)) <= f)
    {
      GD_FERROR("invalid UImageFormat value: %d", f);
      f = IMAGE_UNKNOWN;
    }
    return formats[f];
  }

  UImageFormat
  parse_image_format(const std::string& s)
  {
    for (unsigned i = 0; i < cardinality_of(formats); ++i)
      if (s == formats[i])
        return static_cast<UImageFormat>(i);
    return IMAGE_UNKNOWN;
  }

  /*---------.
  | UImage.  |
  `---------*/

  UImage
  UImage::make()
  {
    UImage res;
    res.data = 0;
    res.size = res.width = res.height = 0;
    res.imageFormat = IMAGE_UNKNOWN;
    return res;
  }

  const char*
  UImage::format_string() const
  {
    return ::urbi::format_string(imageFormat);
  }

  std::string
  UImage::headers_() const
  {
    return libport::format("%s %s %s",
                           format_string(), width, height);
  }

} // namespace urbi
