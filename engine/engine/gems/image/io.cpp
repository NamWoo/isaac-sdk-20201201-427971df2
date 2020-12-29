/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "io.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "jpeglib.h"  // NOLINT
#include "png.h"  // NOLINT

namespace isaac {

namespace {

// Helper to load a PNG
template <typename K, int N>
bool LoadPngImpl(const std::string& filename, Image<K, N>& image) {
  struct Impl {
    std::FILE* fp;
    png_structp png;
    png_infop info;

    ~Impl() {
      png_destroy_read_struct(&png, &info, nullptr);
      std::fclose(fp);
    }
  };

  // Open file
  std::FILE* fp = std::fopen(filename.c_str(), "rb");
  if (fp == nullptr) {
    return false;
  }

  // Store resources in a smart object so that they get automatically deleted
  auto impl = std::make_unique<Impl>();
  impl->fp = fp;

  impl->png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  ASSERT(impl->png, "png_create_read_struct() failed");
  impl->info = png_create_info_struct(impl->png);
  ASSERT(impl->info, "png_create_info_struct() failed");

  ASSERT(setjmp(png_jmpbuf(impl->png)) == 0, "setjmp(png_jmpbuf(png)) failed");

  png_init_io(impl->png, impl->fp);
  png_read_info(impl->png, impl->info);

  const int width = png_get_image_width(impl->png, impl->info);
  const int height = png_get_image_height(impl->png, impl->info);
  const png_byte color_type = png_get_color_type(impl->png, impl->info);
  const png_byte bit_depth = png_get_bit_depth(impl->png, impl->info);
  image.resize(height, width);

  const bool out_color = N > 2;
  const bool in_color = color_type & PNG_COLOR_MASK_COLOR;
  const bool out_alpha = N == 2 || N == 4;
  const bool in_alpha = color_type & PNG_COLOR_MASK_ALPHA;

  if (out_color != in_color) {
    if (out_color) {
      png_set_gray_to_rgb(impl->png);
    } else {
      png_set_rgb_to_gray_fixed(impl->png, 1, 33000, 33000);
    }
  }
  if (out_alpha != in_alpha) {
    if (out_alpha) {
      LOG_ERROR("The loaded PNG does not have an alpha channel, but one was requested");
      return false;
    } else {
      png_set_strip_alpha(impl->png);
    }
  }

  if (bit_depth == 16) {
    if (sizeof(K) == 1) {
      png_set_strip_16(impl->png);
    } else if (typeid(K) != typeid(uint16_t)) {
      LOG_ERROR("16-images can only be loaded into either 8ub or ui16 images");
      return false;
    }
    // TODO Should swap always be used?
    png_set_swap(impl->png);
  }

  if (color_type == PNG_COLOR_TYPE_PALETTE) {
    png_set_palette_to_rgb(impl->png);
  }
  if (bit_depth < 8) {
    png_set_packing(impl->png);
  }

  png_read_update_info(impl->png, impl->info);

  const size_t png_row_bytes = png_get_rowbytes(impl->png, impl->info);
  const size_t image_row_bytes = width * N * sizeof(K);
  ASSERT(png_row_bytes == image_row_bytes, "Dimensions missmatch %d != %d",
         png_row_bytes, image_row_bytes);

  std::vector<png_bytep> row_pointers(height);
  for (int y = 0; y < height; y++) {
    row_pointers[y] = reinterpret_cast<png_bytep>(image.row_pointer(y));
  }
  png_read_image(impl->png, row_pointers.data());

  return true;
}

// Helper to load a JPEG
template <int N>
bool LoadJpegImpl(const std::string& filename, Image<uint8_t, N>& image) {
  const bool out_color = N > 2;
  const bool out_alpha = N == 2 || N == 4;

  if (out_alpha) {
    LOG_ERROR("The JPEG reader does not support an alpha channel, but one was requested");
    return false;
  }

  std::FILE* fp = std::fopen(filename.c_str(), "rb");
  if (fp == nullptr) {
    LOG_ERROR("Could not open file: '%s'", filename.c_str());
    return false;
  }

  // create structures, read .jpeg header
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);

  jpeg_create_decompress(&cinfo);
  jpeg_stdio_src(&cinfo, fp);
  jpeg_read_header(&cinfo, TRUE);

  // Set the image size from dimensions in the .jpeg file
  ASSERT(cinfo.image_height > 0 && cinfo.image_height < 65536, "Invalid JPEG image height.");
  ASSERT(cinfo.image_width > 0 && cinfo.image_width < 65536, "Invalid JPEG image width.");
  image.resize(cinfo.image_height, cinfo.image_width);

  // Set parameters for decompression, defaults are from jpeg_read_header()
  cinfo.out_color_space = out_color ? JCS_RGB : JCS_GRAYSCALE;

  // decompress
  jpeg_start_decompress(&cinfo);

  while (cinfo.output_scanline < cinfo.image_height) {
    JSAMPROW samp = const_cast<uint8_t*>(image.row_pointer(cinfo.output_scanline));
    jpeg_read_scanlines(&cinfo, &samp, 1);
  }

  // release resources
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  std::fclose(fp);
  return true;
}

// Helper to load PNG or JPEG
template <typename K, int N>
bool LoadImageImpl(const std::string& filename, Image<K, N>& image) {
  const std::string lowercased_filename = ToLowerCase(filename);
  if (EndsWith(lowercased_filename, ".png")) {
    if (!LoadPng(filename, image)) {
      return false;
    }
  } else if (EndsWith(lowercased_filename, ".jpg") || EndsWith(lowercased_filename, ".jpeg")) {
    if (!LoadJpeg(filename, image)) {
      return false;
    }
  } else {
    LOG_ERROR("Unrecognized image file %s. Expect: .png|.jpg|.jpeg", filename.c_str());
    return false;
  }
  return true;
}

// Helper to write to a buffer
void PngWriteCallback(png_structp  png_ptr, png_bytep data, png_size_t length) {
    std::vector<uint8_t>* p = (std::vector<uint8_t>*)png_get_io_ptr(png_ptr);
    p->insert(p->end(), data, data + length);
}

// Helper to save a png file
template <typename K, int N>
bool SavePngImpl(const std::string& filename, const ImageConstView<K, N>& image) {
  struct Impl {
    std::FILE* fp;
    png_structp png;
    png_infop info;

    ~Impl() {
      png_destroy_write_struct(&png, &info);
      std::fclose(fp);
    }
  };

  ASSERT(N == 1 || N == 3 || N == 4, "Cannot save an image with %d channels", N);

  std::FILE* fp = std::fopen(filename.c_str(), "wb");
  if (fp == nullptr) {
    LOG_ERROR("Could not open file for writing: %s", filename.c_str());
    return false;
  }

  // Store resources in a smart object so that they get automatically deleted
  auto impl = std::make_unique<Impl>();
  impl->fp = fp;

  impl->png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  ASSERT(impl->png, "png_create_write_struct() failed");
  impl->info = png_create_info_struct(impl->png);
  ASSERT(impl->info, "png_create_info_struct() failed");

  if (setjmp(png_jmpbuf(impl->png)) != 0) {
    LOG_ERROR("setjmp(png_jmpbuf(png)) failed");
    return false;
  }

  png_init_io(impl->png, impl->fp);

  png_set_IHDR(impl->png, impl->info, image.cols(), image.rows(), sizeof(K) * 8,
          (N == 1 ? PNG_COLOR_TYPE_GRAY : (N == 3 ? PNG_COLOR_TYPE_RGB : PNG_COLOR_TYPE_RGBA)),
          PNG_INTERLACE_NONE,
          PNG_COMPRESSION_TYPE_DEFAULT,
          PNG_FILTER_TYPE_DEFAULT);
  png_write_info(impl->png, impl->info);

  if (std::is_same<K, uint16_t>::value) {
    png_set_swap(impl->png);
  }

  png_set_compression_level(impl->png, 1);

  for (int row = 0; row < image.rows(); ++row) {
    png_write_row(impl->png, reinterpret_cast<png_const_bytep>(image.row_pointer(row)));
  }

  png_write_end(impl->png, nullptr);
  return true;
}

// Helper to save a jpg file
template <int N>
bool SaveJpegImpl(const std::string& filename, J_COLOR_SPACE type, int quality,
                  const ImageConstView<uint8_t, N>& image) {
  std::FILE* fp = std::fopen(filename.c_str(), "wb");
  if (fp == nullptr) {
    LOG_ERROR("Could not open the file: %s", filename.c_str());
    return false;
  }

  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  cinfo.image_width = image.cols();
  cinfo.image_height = image.rows();
  cinfo.input_components = N;
  cinfo.in_color_space = type;

  jpeg_stdio_dest(&cinfo, fp);

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  jpeg_start_compress(&cinfo, TRUE);
  while (cinfo.next_scanline < cinfo.image_height) {
    JSAMPROW samp = const_cast<uint8_t*>(image.row_pointer(cinfo.next_scanline));
    jpeg_write_scanlines(&cinfo, &samp, 1);
  }
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  std::fclose(fp);
  return true;
}

// Helper to encode a png
template <int N>
void EncodePngImpl(ImageConstView<uint8_t, N> image, int type, std::vector<uint8_t>& out) {
  png_structp p = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  ASSERT(p, "png_create_write_struct() failed");
  png_infop info_ptr = png_create_info_struct(p);
  ASSERT(info_ptr, "png_create_info_struct() failed");
  ASSERT(0 == setjmp(png_jmpbuf(p)), "setjmp(png_jmpbuf(p) failed");
  png_set_IHDR(p, info_ptr, image.cols(), image.rows(), 8,
          type,
          PNG_INTERLACE_NONE,
          PNG_COMPRESSION_TYPE_DEFAULT,
          PNG_FILTER_TYPE_DEFAULT);
  png_set_compression_level(p, 1);
  std::vector<png_bytep> rows(image.rows());
  for (int y = 0; y < image.rows(); ++y) {
    rows[y] = const_cast<png_bytep>(image.row_pointer(y));
  }
  png_set_rows(p, info_ptr, &rows[0]);
  png_set_write_fn(p, &out, PngWriteCallback, NULL);
  png_write_png(p, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);
  png_destroy_write_struct(&p, &info_ptr);
}

// Helper to encode a jpg image
template <int N>
void EncodeJpegImpl(const ImageConstView<uint8_t, N>& image, J_COLOR_SPACE type, int quality,
                    std::vector<uint8_t>& out) {
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  cinfo.image_width = image.cols();
  cinfo.image_height = image.rows();
  cinfo.input_components = N;
  cinfo.in_color_space = type;

  unsigned char* mem = NULL;
  uint64_t mem_size = 0;
  jpeg_mem_dest(&cinfo, &mem, &mem_size);

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  jpeg_start_compress(&cinfo, TRUE);
  while (cinfo.next_scanline < cinfo.image_height) {
    JSAMPROW samp = const_cast<uint8_t*>(image.row_pointer(cinfo.next_scanline));
    jpeg_write_scanlines(&cinfo, &samp, 1);
  }
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  out.resize(mem_size);
  std::copy(mem, mem + mem_size, out.begin());
  std::free(mem);
}

}  // namespace

bool LoadPng(const std::string& filename, Image1ub& image) {
  return LoadPngImpl(filename, image);
}

bool LoadPng(const std::string& filename, Image1ui16& image) {
  return LoadPngImpl(filename, image);
}

bool LoadPng(const std::string& filename, Image3ub& image) {
  return LoadPngImpl(filename, image);
}

bool LoadPng(const std::string& filename, Image4ub& image) {
  return LoadPngImpl(filename, image);
}

bool LoadJpeg(const std::string& filename, Image3ub& image) {
  return LoadJpegImpl(filename, image);
}

bool LoadJpeg(const std::string& filename, Image1ub& image) {
  return LoadJpegImpl(filename, image);
}

bool LoadImage(const std::string& filename, Image1ub& image) {
  return LoadImageImpl(filename, image);
}

bool LoadImage(const std::string& filename, Image3ub& image) {
  return LoadImageImpl(filename, image);
}

bool SavePng(const ImageConstView1ub& image, const std::string& filename) {
  return SavePngImpl(filename, image);
}

bool SavePng(const ImageConstView3ub& image, const std::string& filename) {
  return SavePngImpl(filename, image);
}

bool SavePng(const ImageConstView4ub& image, const std::string& filename) {
  return SavePngImpl(filename, image);
}

bool SavePng(const ImageConstView1ui16& image, const std::string& filename) {
  return SavePngImpl(filename, image);
}

bool SaveJpeg(const ImageConstView1ub& image, const std::string& filename, const int quality) {
  return SaveJpegImpl(filename, JCS_GRAYSCALE, quality, image);
}

bool SaveJpeg(const ImageConstView3ub& image, const std::string& filename, const int quality) {
  return SaveJpegImpl(filename, JCS_RGB, quality, image);
}

void EncodePng(const ImageConstView1ub& image, std::vector<uint8_t>& encoded) {
  EncodePngImpl(image, PNG_COLOR_TYPE_GRAY, encoded);
}

void EncodePng(const ImageConstView3ub& image, std::vector<uint8_t>& encoded) {
  EncodePngImpl(image, PNG_COLOR_TYPE_RGB, encoded);
}

void EncodePng(const ImageConstView4ub& image, std::vector<uint8_t>& encoded) {
  EncodePngImpl(image, PNG_COLOR_TYPE_RGBA, encoded);
}

void EncodeJpeg(const ImageConstView1ub& image, int quality, std::vector<uint8_t>& encoded) {
  EncodeJpegImpl(image, JCS_GRAYSCALE, quality, encoded);
}

void EncodeJpeg(const ImageConstView3ub& image, int quality, std::vector<uint8_t>& encoded) {
  EncodeJpegImpl(image, JCS_RGB, quality, encoded);
}

}  // namespace isaac
