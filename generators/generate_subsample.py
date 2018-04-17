from generators.generate_pybind11_bindings import generate, write_stuff_if_needed, get_headers


def main():
    modules = ["ml"]
    headers = get_headers(modules)
    headers = [
        ("", "pcl_base.h", ""),
    #     ("features", "feature.h", ""),
    ]
    generated_headers = generate(headers)
    write_stuff_if_needed(generated_headers, delete_others=True)


if __name__ == '__main__':
    main()
