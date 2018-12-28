from conans import ConanFile, CMake, tools


class GraphenepclConan(ConanFile):
    name = "graphene-pcl"
    version = "0.1"
    license = "unlicense"
    author = "Richard Vock vock@cs.uni-bonn.de"
    url = "https://github.com/richard-vock/graphene-pcl"
    description = "Graphene extension to load pointclouds in PCD format"
    topics = ("rendering", "visualization", "pointclouds")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False]}
    default_options = "shared=False"
    generators = "cmake"
    exports_sources = "include*", "src*", "CMakeLists.txt"
    requires = (("graphene/0.1@richard-vock/dev"),
                ("CTRE/2.0@richard-vock/dev", "private"))

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        self.copy("include/graphene/*.hpp")
        self.copy("*graphene-pcl.lib", dst="lib", keep_path=False)
        self.copy("*.dll", dst="bin", keep_path=False)
        self.copy("*.so", dst="lib", keep_path=False)
        self.copy("*.dylib", dst="lib", keep_path=False)
        self.copy("*.a", dst="lib", keep_path=False)

    def package_info(self):
        self.cpp_info.libs = ["graphene-pcl"]

