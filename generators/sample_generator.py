from generators.main_generator import generate, write_stuff_if_needed, get_headers


def main():
    # modules = ["filters"]
    # headers = get_headers(modules)
    headers = [
        ("", "pcl_base.h", ""),
        ("filters", "filter.h", ""),
        ("filters", "fast_bilateral.h", ""),
        ("filters", "median_filter.h", ""),
        ("filters", "convolution.h", ""),
        ("filters", "fast_bilateral_omp.h", ""),
    ]
    generated_headers = generate(headers)
    write_stuff_if_needed(generated_headers, delete_others=False)


if __name__ == '__main__':
    main()
