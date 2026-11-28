"""Locate the development headers for the selected Python interpreter."""

def _python_headers_impl(ctx):
    python = ctx.getenv("MOTORSIM_PYTHON")
    if not python:
        fail("Build the Python extension with scripts/build_python.sh")
    result = ctx.execute([python, "-c", """
import sysconfig
import pybind11

print(sysconfig.get_path("include"))
print(sysconfig.get_path("platinclude"))
print(pybind11.get_include())
"""])
    if result.return_code:
        fail("Could not locate Python and pybind11 headers: " + result.stderr)

    include, platinclude, pybind = result.stdout.strip().split("\n")
    ctx.symlink(include, "python_include")
    ctx.symlink(pybind, "pybind_include")
    includes = ["python_include", "pybind_include"]
    if platinclude != include:
        ctx.symlink(platinclude, "platform_include")
        includes.append("platform_include")
    ctx.file("BUILD.bazel", """
load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "headers",
    hdrs = glob(%s),
    includes = %s,
    visibility = ["//visibility:public"],
)
""" % ([path + "/**/*.h" for path in includes], includes))

python_headers = repository_rule(implementation = _python_headers_impl, local = True)
