/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_internal.hpp"

#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <limits.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace openarm::runtime {
namespace {

bool absolute_regular_path(const char *path, std::string &directory, std::string &name) {
    if (path == nullptr) return false;
    const std::string value(path);
    if (value.empty() || value.size() >= PATH_MAX || value.front() != '/' ||
        value.back() == '/' || value.find("//") != std::string::npos ||
        value.find("/./") != std::string::npos || value.find("/../") != std::string::npos) {
        return false;
    }
    const std::size_t slash = value.rfind('/');
    directory = slash == 0U ? "/" : value.substr(0U, slash);
    name = value.substr(slash + 1U);
    if (name.empty() || name == "." || name == ".." || name.size() > 200U) return false;
    std::string component;
    for (std::size_t i = 1U; i <= directory.size(); ++i) {
        if (i == directory.size() || directory[i] == '/') {
            component = directory.substr(0U, i == 1U ? 1U : i);
            struct stat status{};
            if (lstat(component.c_str(), &status) != 0 || !S_ISDIR(status.st_mode) ||
                S_ISLNK(status.st_mode)) return false;
        }
    }
    const std::string target = directory + (directory == "/" ? "" : "/") + name;
    struct stat status{};
    if (lstat(target.c_str(), &status) == 0 && (!S_ISREG(status.st_mode) || S_ISLNK(status.st_mode))) {
        return false;
    }
    return errno == ENOENT || S_ISREG(status.st_mode);
}

oa_runtime_status read_file(const char *path, std::string &text) {
    std::string directory;
    std::string name;
    if (!absolute_regular_path(path, directory, name)) return OA_RUNTIME_EPERMISSION;
    const int directory_fd = open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (directory_fd < 0) return OA_RUNTIME_EIO;
    const int fd = openat(directory_fd, name.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) {
        close(directory_fd);
        return OA_RUNTIME_EIO;
    }
    struct stat status{};
    if (fstat(fd, &status) != 0 || !S_ISREG(status.st_mode) || status.st_size <= 0 ||
        status.st_size > 65536) {
        close(fd);
        close(directory_fd);
        return OA_RUNTIME_ECORRUPT;
    }
    try {
        text.assign(static_cast<std::size_t>(status.st_size), '\0');
    } catch (...) {
        close(fd);
        close(directory_fd);
        return OA_RUNTIME_ENOMEM;
    }
    std::size_t offset = 0U;
    while (offset < text.size()) {
        const ssize_t count = read(fd, text.data() + offset, text.size() - offset);
        if (count <= 0) {
            close(fd);
            close(directory_fd);
            return OA_RUNTIME_EIO;
        }
        offset += static_cast<std::size_t>(count);
    }
    std::uint8_t extra = 0U;
    if (read(fd, &extra, 1U) != 0) {
        close(fd);
        close(directory_fd);
        return OA_RUNTIME_ECORRUPT;
    }
    close(fd);
    close(directory_fd);
    return OA_RUNTIME_OK;
}

}
}

extern "C" oa_runtime_status oa_runtime_manifest_load(
    const char *absolute_path, oa_runtime_manifest **out_manifest) {
    if (out_manifest == nullptr) return OA_RUNTIME_EINVAL;
    *out_manifest = nullptr;
    try {
        std::string text;
        oa_runtime_status status = openarm::runtime::read_file(absolute_path, text);
        if (status != OA_RUNTIME_OK) return status;
        std::shared_ptr<openarm::runtime::ManifestData> manifest;
        status = openarm::runtime::parse_manifest(text, manifest);
        if (status != OA_RUNTIME_OK) return status;
        oa_runtime_manifest *const handle = openarm::runtime::manifests.insert(manifest);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
        *out_manifest = handle;
        return OA_RUNTIME_OK;
    } catch (...) {
        return OA_RUNTIME_ENOMEM;
    }
}

extern "C" oa_runtime_status oa_runtime_manifest_preview_file(
    const char *absolute_path, oa_runtime_manifest_preview *out_preview) {
    if (!openarm::runtime::output_valid(out_preview)) return OA_RUNTIME_EABI;
    std::string text;
    oa_runtime_status status = openarm::runtime::read_file(absolute_path, text);
    if (status != OA_RUNTIME_OK) return status;
    std::shared_ptr<openarm::runtime::ManifestData> manifest;
    status = openarm::runtime::parse_manifest(text, manifest);
    if (status != OA_RUNTIME_OK) return status;
    oa_runtime_manifest_preview result{};
    openarm::runtime::runtime_init(result);
    result.valid = 1U;
    result.changed_motor_mask = 0U;
    result.would_be_armable = manifest->state == OA_RUNTIME_MANIFEST_ARMABLE ? 1U : 0U;
    result.base_revision = manifest->config.manifest_revision;
    result.result_revision = manifest->config.manifest_revision;
    result.validation_status = OA_RUNTIME_OK;
    *out_preview = result;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_manifest_save(
    const oa_runtime_manifest *manifest, const char *absolute_path,
    std::uint32_t persistence_authority) {
    if (persistence_authority != OA_RUNTIME_PERSISTENCE_AUTHORIZED) {
        return OA_RUNTIME_EPERMISSION;
    }
    const auto pinned = openarm::runtime::manifests.pin(manifest);
    if (!pinned) return OA_RUNTIME_EINVAL;
    try {
        std::string directory;
        std::string name;
        if (!openarm::runtime::absolute_regular_path(absolute_path, directory, name)) {
            return OA_RUNTIME_EPERMISSION;
        }
        openarm::runtime::ManifestData copy = *pinned;
        copy.content_digest = openarm::runtime::sha256_hex(
            openarm::runtime::manifest_canonical(copy, false));
        const std::string contents = openarm::runtime::manifest_canonical(copy, true);
        const int directory_fd = open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
        if (directory_fd < 0) return OA_RUNTIME_EIO;
        static std::atomic<std::uint64_t> sequence{1U};
        std::string temporary;
        int fd = -1;
        for (unsigned attempt = 0U; attempt < 64U && fd < 0; ++attempt) {
            temporary = ".openarm-runtime-" + std::to_string(getpid()) + "-" +
                        std::to_string(sequence.fetch_add(1U, std::memory_order_relaxed));
            fd = openat(directory_fd, temporary.c_str(),
                        O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
            if (fd < 0 && errno != EEXIST) break;
        }
        if (fd < 0) {
            close(directory_fd);
            return OA_RUNTIME_EIO;
        }
        std::size_t offset = 0U;
        bool success = true;
        while (offset < contents.size()) {
            const ssize_t count = write(fd, contents.data() + offset, contents.size() - offset);
            if (count <= 0) { success = false; break; }
            offset += static_cast<std::size_t>(count);
        }
        if (success && fsync(fd) != 0) success = false;
        if (close(fd) != 0) success = false;
        if (success && renameat(directory_fd, temporary.c_str(), directory_fd, name.c_str()) != 0) {
            success = false;
        }
        if (success && fsync(directory_fd) != 0) success = false;
        if (!success) unlinkat(directory_fd, temporary.c_str(), 0);
        close(directory_fd);
        return success ? OA_RUNTIME_OK : OA_RUNTIME_EIO;
    } catch (...) {
        return OA_RUNTIME_ENOMEM;
    }
}
