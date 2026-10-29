// Offline file import and planning only. No Robot, COM, ADS, Teknic or head driver.
#ifndef _WIN32
#error The S2-A CLI filesystem implementation requires Windows.
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif
#include <windows.h>

#include "cdpr_slicer/Planner.h"
#include "cdpr_slicer/Mechanics.h"
#include "json.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <new>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifndef CDPR_SLICER_PROJECT_ROOT
#error CMake must supply the slicer project root.
#endif

namespace {
using namespace cdpr_slicer;
using Json = nlohmann::json;
constexpr std::size_t kMaximumJobBytes = 64u * 1024u * 1024u;

void Require(bool condition, const char* message) {
    if(!condition) throw std::runtime_error(message);
}
void WindowsFailure(const char* action) {
    throw std::runtime_error(std::string(action) + " (Windows error " + std::to_string(GetLastError()) + ").");
}
class Handle {
    HANDLE value_{INVALID_HANDLE_VALUE};
public:
    Handle() = default;
    explicit Handle(HANDLE value) : value_(value) {}
    ~Handle() { if(value_ != INVALID_HANDLE_VALUE) CloseHandle(value_); }
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
    Handle(Handle&& other) noexcept : value_(other.value_) { other.value_ = INVALID_HANDLE_VALUE; }
    Handle& operator=(Handle&& other) noexcept {
        if(this != &other) {
            if(value_ != INVALID_HANDLE_VALUE) CloseHandle(value_);
            value_ = other.value_; other.value_ = INVALID_HANDLE_VALUE;
        }
        return *this;
    }
    HANDLE get() const { return value_; }
};
std::wstring Wide(const std::string& text) {
    Require(!text.empty() && text.size() <= 32767, "Invalid UTF-8 path length.");
    const auto n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()), nullptr, 0);
    Require(n > 0, "Invalid UTF-8 path.");
    std::wstring out(static_cast<std::size_t>(n), L'\0');
    Require(MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()), &out[0], n) == n, "Path conversion failed.");
    return out;
}
std::string Utf8(const std::wstring& text) {
    Require(!text.empty() && text.size() <= 32767, "Invalid path length.");
    const auto n = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
    Require(n > 0, "Invalid Unicode path.");
    std::string out(static_cast<std::size_t>(n), '\0');
    Require(WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, text.data(), static_cast<int>(text.size()), &out[0], n, nullptr, nullptr) == n, "Path conversion failed.");
    return out;
}
bool DrivePath(const std::wstring& path) {
    return path.size() >= 3 && ((path[0] >= L'A' && path[0] <= L'Z') || (path[0] >= L'a' && path[0] <= L'z'))
        && path[1] == L':' && path[2] == L'\\';
}
std::wstring FullPath(std::wstring path) {
    Require(!path.empty(), "A path is required.");
    std::replace(path.begin(), path.end(), L'/', L'\\');
    Require(path[0] != L'\\', "UNC, device and root-relative paths are not supported.");
    Require(path.find(L':') == std::wstring::npos || DrivePath(path), "Drive-relative paths and alternate streams are forbidden.");
    for(std::size_t i = 0; i < path.size(); ++i)
        Require(path[i] >= 32 && (path[i] != L':' || i == 1), "Control characters and alternate streams are forbidden in paths.");
    std::vector<wchar_t> buffer(32768);
    const auto n = GetFullPathNameW(path.c_str(), static_cast<DWORD>(buffer.size()), buffer.data(), nullptr);
    Require(n > 0 && n < buffer.size(), "Cannot normalize path.");
    std::wstring full(buffer.data(), n);
    Require(DrivePath(full) && full.size() < 240, "Use a conventional local drive path shorter than 240 characters.");
    if(full.size() > 3 && full.back() == L'\\') full.pop_back();
    const auto drive_type = GetDriveTypeW(full.substr(0,3).c_str());
    Require(drive_type == DRIVE_FIXED || drive_type == DRIVE_REMOVABLE, "Use a local disk drive.");
    std::size_t at = 3;
    while(at < full.size()) {
        const auto end = full.find(L'\\', at);
        auto part = full.substr(at, end == std::wstring::npos ? end : end - at);
        Require(!part.empty() && part.back() != L'.' && part.back() != L' ', "Ambiguous trailing dots/spaces are forbidden in paths.");
        part = part.substr(0,part.find(L'.'));
        for(auto& c : part) if(c >= L'a' && c <= L'z') c = static_cast<wchar_t>(c - L'a' + L'A');
        Require(part != L"CON" && part != L"PRN" && part != L"AUX" && part != L"NUL"
            && part != L"CLOCK$" && part != L"CONIN$" && part != L"CONOUT$", "Reserved device names are forbidden.");
        if(part.size() == 4 && (part.substr(0,3) == L"COM" || part.substr(0,3) == L"LPT"))
            Require(!((part[3] >= L'0' && part[3] <= L'9') || part[3] == L'\x00B9' || part[3] == L'\x00B2' || part[3] == L'\x00B3'),
                "Reserved port names are forbidden.");
        if(end == std::wstring::npos) break;
        at = end + 1;
    }
    return full;
}
// Force literal local file semantics at the API boundary, including names which
// Win32 would otherwise normalize into devices. User-supplied prefixes are rejected.
std::wstring LiteralPath(const std::wstring& full) { return L"\\\\?\\" + full; }
std::wstring Parent(const std::wstring& path) {
    const auto at = path.find_last_of(L'\\'); Require(at != std::wstring::npos, "Path has no parent.");
    return path.substr(0, at == 2 ? 3 : at);
}
std::wstring Leaf(const std::wstring& path) { return path.substr(path.find_last_of(L'\\') + 1); }
std::wstring FromDirectory(const std::wstring& directory, const std::wstring& path) {
    if(path.size() > 1 && path[1] == L':') return FullPath(path);
    Require(!path.empty() && path[0] != L'\\' && path[0] != L'/', "Use a local absolute or relative path.");
    return FullPath(directory + L"\\" + path);
}
bool SamePath(const std::wstring& a, const std::wstring& b) {
    return CompareStringOrdinal(a.c_str(), static_cast<int>(a.size()), b.c_str(), static_cast<int>(b.size()), TRUE) == CSTR_EQUAL;
}
void DiskInformation(HANDLE file, bool directory) {
    Require(GetFileType(file) == FILE_TYPE_DISK, "Only local disk files/directories are supported.");
    BY_HANDLE_FILE_INFORMATION info{};
    if(!GetFileInformationByHandle(file, &info)) WindowsFailure("Cannot inspect file");
    Require((info.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) == 0, "Reparse points are not supported.");
    Require(((info.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0) == directory, "Unexpected file/directory type.");
    std::vector<wchar_t> buffer(32768);
    const auto n = GetFinalPathNameByHandleW(file, buffer.data(), static_cast<DWORD>(buffer.size()), FILE_NAME_NORMALIZED | VOLUME_NAME_DOS);
    Require(n > 0 && n < buffer.size(), "Cannot resolve final local disk path.");
    const std::wstring resolved(buffer.data(), n);
    Require(resolved.compare(0, 4, L"\\\\?\\") == 0 && DrivePath(resolved.substr(4)), "Network/device targets are forbidden.");
}
std::string ReadRegularFile(const std::wstring& path, std::size_t maximum) {
    Handle file(CreateFileW(LiteralPath(path).c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING, FILE_FLAG_OPEN_REPARSE_POINT, nullptr));
    if(file.get() == INVALID_HANDLE_VALUE) WindowsFailure("Cannot open input");
    DiskInformation(file.get(), false);
    LARGE_INTEGER size{};
    if(!GetFileSizeEx(file.get(), &size)) WindowsFailure("Cannot read input size");
    Require(size.QuadPart >= 0 && static_cast<unsigned long long>(size.QuadPart) <= maximum, "Input byte limit exceeded.");
    std::string bytes(static_cast<std::size_t>(size.QuadPart), '\0');
    std::size_t offset = 0;
    while(offset < bytes.size()) {
        DWORD read = 0;
        const auto requested = static_cast<DWORD>(std::min<std::size_t>(bytes.size() - offset, 65536));
        if(!ReadFile(file.get(), &bytes[offset], requested, &read, nullptr)) WindowsFailure("Cannot read input");
        Require(read != 0, "Input ended unexpectedly."); offset += read;
    }
    return bytes;
}
Handle LockDirectory(const std::wstring& path) {
    // Excluding FILE_SHARE_DELETE holds the directory in place until publication.
    Handle directory(CreateFileW(LiteralPath(path).c_str(), FILE_READ_ATTRIBUTES, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
        FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OPEN_REPARSE_POINT, nullptr));
    if(directory.get() == INVALID_HANDLE_VALUE) WindowsFailure("Cannot hold output directory");
    DiskInformation(directory.get(), true); return directory;
}
std::vector<Handle> LockDirectoryChain(const std::wstring& full) {
    std::vector<Handle> held;
    held.push_back(LockDirectory(full.substr(0,3)));
    std::size_t at = 3;
    while(at < full.size()) {
        const auto end = full.find(L'\\', at);
        held.push_back(LockDirectory(full.substr(0,end)));
        if(end == std::wstring::npos) break;
        at = end + 1;
    }
    return held;
}
bool OutputName(const std::wstring& name,bool html=false) {
    if(name.size() < 6 || name.size() > 120 || name.substr(name.size() - 5) != (html?L".html":L".json")) return false;
    const auto stem = name.substr(0, name.size() - 5);
    if(stem.empty()) return false;
    for(wchar_t c : stem)
        if(!((c >= L'A' && c <= L'Z') || (c >= L'a' && c <= L'z') || (c >= L'0' && c <= L'9') || c == L'-' || c == L'_')) return false;
    std::wstring upper = stem;
    for(auto& c : upper) if(c >= L'a' && c <= L'z') c = static_cast<wchar_t>(c - L'a' + L'A');
    if(upper == L"CON" || upper == L"PRN" || upper == L"AUX" || upper == L"NUL") return false;
    if(upper.size() == 4 && (upper.substr(0,3) == L"COM" || upper.substr(0,3) == L"LPT") && upper[3] >= L'0' && upper[3] <= L'9') return false;
    return true;
}
std::wstring WriteNewArtifact(const std::wstring& project,const std::wstring& name,const std::string& text,bool html=false) {
    Require(OutputName(name,html),"Use a simple non-reserved filename with the required .json/.html suffix.");
    const auto output = FullPath(project + L"\\out");
    const auto target = FullPath(output + L"\\" + name);
    auto held = LockDirectoryChain(project);
    if(!CreateDirectoryW(LiteralPath(output).c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS) WindowsFailure("Cannot create project out directory");
    auto output_handle = LockDirectory(output);
    Handle file;
    for(unsigned attempt = 0; attempt < 32; ++attempt) {
        const auto temporary = FullPath(output + L"\\pending-" + std::to_wstring(GetCurrentProcessId()) + L"-" + std::to_wstring(attempt) + L".tmp");
        file = Handle(CreateFileW(LiteralPath(temporary).c_str(), GENERIC_WRITE | DELETE, 0, nullptr, CREATE_NEW,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OPEN_REPARSE_POINT, nullptr));
        if(file.get() != INVALID_HANDLE_VALUE) break;
        if(GetLastError() != ERROR_FILE_EXISTS && GetLastError() != ERROR_ALREADY_EXISTS) WindowsFailure("Cannot create new import artifact");
    }
    Require(file.get() != INVALID_HANDLE_VALUE, "No unused temporary artifact name is available.");
    try {
        DiskInformation(file.get(), false);
        std::size_t offset = 0;
        while(offset < text.size()) {
            DWORD written = 0;
            const auto count = static_cast<DWORD>(std::min<std::size_t>(text.size() - offset, 65536));
            if(!WriteFile(file.get(), text.data() + offset, count, &written, nullptr)) WindowsFailure("Cannot write import artifact");
            Require(written != 0, "Artifact write made no progress."); offset += written;
        }
        if(!FlushFileBuffers(file.get())) WindowsFailure("Cannot flush import artifact");
        // Publish the already-open file by handle. ReplaceIfExists=FALSE is
        // enforced by the filesystem; there is no check-then-overwrite window.
        const auto rename_bytes = offsetof(FILE_RENAME_INFO, FileName) + (target.size() + 1) * sizeof(wchar_t);
        std::unique_ptr<unsigned char[]> storage(new unsigned char[rename_bytes]());
        auto* rename = new (storage.get()) FILE_RENAME_INFO{};
        rename->ReplaceIfExists = FALSE; rename->RootDirectory = nullptr;
        rename->FileNameLength = static_cast<DWORD>(target.size() * sizeof(wchar_t));
        std::memcpy(rename->FileName, target.c_str(), (target.size() + 1) * sizeof(wchar_t));
        if(!SetFileInformationByHandle(file.get(), FileRenameInfo, rename, static_cast<DWORD>(rename_bytes)))
            WindowsFailure("Cannot publish artifact; an existing output is never replaced");
    } catch(...) {
        FILE_DISPOSITION_INFO discard{}; discard.DeleteFile = TRUE;
        // Best effort cleanup targets only the newly created, still-open file.
        if(!SetFileInformationByHandle(file.get(), FileDispositionInfo, &discard, sizeof(discard)))
            std::cerr << "A pending temporary file remains under out/; no successful artifact was published.\n";
        throw;
    }
    return target;
}
Json Configuration(const std::string& text,bool planning) {
    std::vector<std::set<std::string>> keys(34);
    auto callback = [&](int depth, Json::parse_event_t event, Json& value) {
        Require(depth >= 0 && depth <= 32, "Configuration nesting limit exceeded.");
        if(event == Json::parse_event_t::object_start) keys[static_cast<std::size_t>(depth) + 1].clear();
        if(event == Json::parse_event_t::key)
            Require(keys[static_cast<std::size_t>(depth)].insert(value.get<std::string>()).second, "Duplicate configuration key.");
        return true;
    };
    const auto config = Json::parse(text, callback, true, false);
    const std::set<std::string> required{"schema_version","mode","hardware_access","robot_model","import","frames","planning","output_directory"};
    Require(config.is_object() && config.size() == required.size(), "Missing or extra configuration fields.");
    for(const auto& key : required) Require(config.contains(key), "Missing configuration field.");
    Require(config.at("schema_version").is_number_integer() && config.at("schema_version") == 1, "Unsupported configuration version.");
    Require(config.at("mode").is_string() && config.at("mode") == "offline_planning", "Only offline_planning mode is supported.");
    Require(config.at("hardware_access").is_boolean() && config.at("hardware_access") == false, "hardware_access must be false.");
    Require(config.at("robot_model").is_object() && config.at("planning").is_object(), "Expected reserved robot_model/planning objects.");
    const Json default_frames = {{"world_frame_id","cdpr_world"},{"bed_frame_id","print_bed"},{"end_effector_frame_id","end_effector"},
        {"nozzle_frame_id","nozzle_tip"},{"world_from_bed",nullptr},{"end_effector_from_nozzle",nullptr},{"status","unverified"}};
    if(!planning) Require(config.at("frames")==default_frames,"Import-only mode requires default frame names and no registration.");
    else Require(DeserializeFrameRegistration(config.at("frames").dump()).status==ResultStatus::success,"Invalid planning frame configuration.");
    Require(config.at("output_directory").is_string(), "output_directory must be a path string.");
    return config;
}
ModelLoadRequest ModelRequest(const Json& model,const std::wstring& config_directory) {
    const std::set<std::string> fields{"configuration_path","expected_robot_config_sha256","calibration_manifest_path",
        "motion_limits_manifest_path","rail_offsets_m","starting_rail_offsets_m"};
    Require(model.is_object()&&model.size()==fields.size(),"Missing or extra robot_model fields.");
    for(const auto& key:fields) Require(model.contains(key),"Missing robot_model field.");
    auto path=[&](const Json& value){Require(value.is_string(),"A model path must be a string.");return Utf8(FromDirectory(config_directory,Wide(value.get<std::string>())));};
    ModelLoadRequest request; request.robot_config_path=path(model.at("configuration_path"));
    if(!model.at("expected_robot_config_sha256").is_null()) {
        Require(model.at("expected_robot_config_sha256").is_string(),"Expected model hash must be a string or null.");
        request.expected_robot_config_sha256.has_value=true; request.expected_robot_config_sha256.value=model.at("expected_robot_config_sha256").get<std::string>();
    }
    if(!model.at("calibration_manifest_path").is_null()) {
        request.calibration_manifest_path.has_value=true; request.calibration_manifest_path.value=path(model.at("calibration_manifest_path"));
    }
    if(!model.at("motion_limits_manifest_path").is_null()) {
        request.motion_limits_manifest_path.has_value=true; request.motion_limits_manifest_path.value=path(model.at("motion_limits_manifest_path"));
    }
    auto rails=[](const Json& value){
        OptionalValue<std::array<double,4>> result;
        if(!value.is_null()) {
            Require(value.is_array()&&value.size()==4,"Explicit four-element rail arrays are required.");
            result.has_value=true;
            for(std::size_t i=0;i<4;++i) {
                Require(value.at(i).is_number(),"Rail offsets must be numbers."); result.value[i]=value.at(i).get<double>();
                Require(std::isfinite(result.value[i]),"Rail offsets must be finite.");
            }
        } return result;};
    request.rail_offsets_m=rails(model.at("rail_offsets_m")); request.starting_rail_offsets_m=rails(model.at("starting_rail_offsets_m"));
    return request;
}
void PrintDiagnostics(const std::vector<Diagnostic>& diagnostics) {
    for(const auto& d : diagnostics) {
        // JSON quoting prevents messages/source metadata from becoming terminal controls.
        std::cerr << Json(d.message).dump();
        if(d.source.has_value && !d.source.value.synthetic) std::cerr << " at line " << d.source.value.first_line;
        std::cerr << '\n';
    }
}
template<typename T> bool Usable(const Result<T>& result) {
    if(result.status == ResultStatus::success) return true;
    PrintDiagnostics(result.diagnostics); return false;
}
void Summary(const PrintJob& job) {
    const char* stage=job.stage==PlanningStage::imported?"imported":job.stage==PlanningStage::transformed?"transformed"
        :job.stage==PlanningStage::geometrically_checked?"geometrically_checked":"timed";
    std::size_t passed=0,unknown=0;
    for(const auto& check:job.validation.checks) {
        if(check.status==ValidationStatus::passed) ++passed;
        if(check.status==ValidationStatus::not_evaluated) ++unknown;
    }
    std::cout<<"job_id="<<Json(job.job_id).dump()<<" stage="<<stage<<" operations="<<job.operations.size()<<'\n'
        <<"hardware_ready=false; "<<passed<<" recorded checks passed, "<<unknown<<" remain not_evaluated.\n";
}
void Help() {
    std::cout << "Offline G-code import and bounded point-to-point planning\n"
        "  cdpr-slicer-plan --input <gcode> --config <planner.json> [--stage <stage>]\n"
        "                   [--output <name.json>] [--preview-output <name.html>]\n"
        "  cdpr-slicer-plan --inspect-job <job.json>\n"
        "  cdpr-slicer-plan --assess-job <job.json> --config <planner.json>\n"
        "                   --physics <mechanics.json> --output <assessment.json>\n"
        "                   [--preview-output <name.html>]\n"
        "  cdpr-slicer-plan --help\n"
        "Stages: imported (default), transformed, geometrically_checked, timed.\n"
        "Input and configured output paths are relative to the configuration directory.\n"
        "Output is restricted to new files in this checkout's CDPR_3D_Printing_Slicer/out/.\n"
        "The default imported stage does not read robot_model files or apply planning settings.\n"
        "The timed stage requires explicit rail state, calibration/limit manifests and sampling settings.\n";
}
} // namespace

int wmain(int argc, wchar_t** argv) {
    try {
        if(argc == 2 && std::wstring(argv[1]) == L"--help") { Help(); return 0; }
        if(argc == 3 && std::wstring(argv[1]) == L"--inspect-job") {
            const auto read = DeserializePrintJob(ReadRegularFile(FullPath(argv[2]),kMaximumJobBytes));
            if(!Usable(read)) return read.status == ResultStatus::fault ? 3 : 2;
            Summary(read.value); return 0;
        }
        if(argc>1&&std::wstring(argv[1])==L"--assess-job") {
            std::wstring job_path,config_path,physics_path,output_name,preview_name;std::set<std::wstring> seen;
            for(int i=1;i<argc;i+=2) {
                const std::wstring flag=argv[i];Require(i+1<argc&&seen.insert(flag).second,"Each mechanics option requires one value and may appear once.");
                if(flag==L"--assess-job") job_path=argv[i+1];
                else if(flag==L"--config") config_path=argv[i+1];
                else if(flag==L"--physics") physics_path=argv[i+1];
                else if(flag==L"--output") output_name=argv[i+1];
                else if(flag==L"--preview-output") preview_name=argv[i+1];
                else throw std::runtime_error("Unknown mechanics option.");
            }
            Require(!job_path.empty()&&!config_path.empty()&&!physics_path.empty()&&OutputName(output_name),"Mechanics assessment requires --assess-job, --config, --physics and a simple --output JSON name.");
            Require(preview_name.empty()||OutputName(preview_name,true),"Invalid mechanics preview name.");
            config_path=FullPath(config_path);const auto config=Configuration(ReadRegularFile(config_path,1024u*1024u),true);
            const auto project=FullPath(Wide(CDPR_SLICER_PROJECT_ROOT));
            Require(SamePath(FromDirectory(Parent(config_path),Wide(config.at("output_directory").get<std::string>())),FullPath(project+L"\\out")),"output_directory must resolve to this slicer project's out directory.");
            const auto loaded=DeserializePrintJob(ReadRegularFile(FullPath(job_path),kMaximumJobBytes));if(!Usable(loaded)) return 2;
            const auto model=LoadRobotModel(ModelRequest(config.at("robot_model"),Parent(config_path)));if(!Usable(model)) return 2;
            const auto options=DeserializePlanningOptions(Json{{"frames",config.at("frames")},{"planning",config.at("planning")}}.dump());if(!Usable(options)) return 2;
            const auto bound=BindPlanningOptions(options.value,model.value);if(!Usable(bound)) return 2;
            const auto assessment=AssessMechanics(loaded.value,model.value,ReadRegularFile(FullPath(physics_path),4u*1024u*1024u));if(!Usable(assessment)) return 2;
            OptionalValue<std::string> preview;
            if(!preview_name.empty()) {const auto rendered=CreatePreviewHtml(loaded.value,model.value,assessment.value);if(!Usable(rendered)) return 2;preview.has_value=true;preview.value=rendered.value;}
            const auto written=WriteNewArtifact(project,output_name,assessment.value);const auto report=Json::parse(assessment.value);
            std::cout<<"Mechanics: collision="<<report.at("collision").at("status")<<" tension="<<report.at("tension").at("status")<<" samples="<<report.at("sample_count")<<" hardware_ready=false\n";
            std::cout<<"Created "<<Json(Utf8(written)).dump()<<'\n';
            if(preview.has_value) std::cout<<"Created "<<Json(Utf8(WriteNewArtifact(project,preview_name,preview.value,true))).dump()<<'\n';
            return 0;
        }
        std::wstring input,config_path,output_name,preview_name,stage=L"imported";
        std::set<std::wstring> seen;
        for(int i = 1; i < argc; i += 2) {
            const std::wstring flag = argv[i];
            Require(i + 1 < argc && seen.insert(flag).second, "Each option requires one value and may appear once.");
            if(flag == L"--input") input = argv[i+1];
            else if(flag == L"--config") config_path = argv[i+1];
            else if(flag == L"--output") output_name = argv[i+1];
            else if(flag == L"--preview-output") preview_name=argv[i+1];
            else if(flag == L"--stage") stage=argv[i+1];
            else { Help(); throw std::runtime_error("Unknown option."); }
        }
        Require(!input.empty() && !config_path.empty(), "Both --input and --config are required. Use --help for syntax.");
        Require(seen.count(L"--output") == 0 || !output_name.empty(), "--output cannot be empty.");
        Require(seen.count(L"--preview-output")==0||!preview_name.empty(),"--preview-output cannot be empty.");
        Require(stage==L"imported"||stage==L"transformed"||stage==L"geometrically_checked"||stage==L"timed","Unknown planning stage.");
        Require(output_name.empty()||OutputName(output_name),"Invalid JSON output filename.");
        Require(preview_name.empty()||OutputName(preview_name,true),"Invalid HTML preview filename.");
        config_path = FullPath(config_path);
        const auto config_bytes = ReadRegularFile(config_path,1024u * 1024u);
        const auto config = Configuration(config_bytes,stage!=L"imported");
        const auto options = DeserializeImportOptions(config.at("import").dump());
        if(!Usable(options)) return 2;
        const auto project = FullPath(Wide(CDPR_SLICER_PROJECT_ROOT));
        const auto output = FromDirectory(Parent(config_path),Wide(config.at("output_directory").get<std::string>()));
        Require(SamePath(output,FullPath(project + L"\\out")), "output_directory must resolve to this slicer project's out directory.");
        input = FromDirectory(Parent(config_path),input);
        GCodeParseRequest request;
        request.options = options.value; request.text = ReadRegularFile(input,request.options.maximum_input_bytes);
        request.source.file_name = Utf8(Leaf(input));
        const auto profile = ComputeSourceSha256(config_bytes);
        if(!Usable(profile)) return 3;
        request.source.profile_sha256.has_value = true; request.source.profile_sha256.value = profile.value;
        const auto parsed = ParseGCode(request);
        if(!Usable(parsed)) return parsed.status == ResultStatus::fault ? 3 : 2;
        auto job=parsed.value;
        OptionalValue<RobotModelSnapshot> preview_model;
        if(stage!=L"imported") {
            const auto planning=DeserializePlanningOptions(Json{{"frames",config.at("frames")},{"planning",config.at("planning")}}.dump());
            if(!Usable(planning)) return 2;
            const auto model=LoadRobotModel(ModelRequest(config.at("robot_model"),Parent(config_path)));
            if(!Usable(model)) return model.status==ResultStatus::fault?3:2;
            PrintDiagnostics(model.diagnostics);
            Result<PrintJob> planned;
            if(stage==L"timed") planned=PlanTrajectory(job,model.value,planning.value);
            else if(stage==L"geometrically_checked") planned=CreateGeometricPlan(job,model.value,planning.value);
            else {
                const auto bound=BindPlanningOptions(planning.value,model.value);
                if(!Usable(bound)) return 2;
                planned=TransformToolpath(job,bound.value);
            }
            if(!Usable(planned)) return planned.status==ResultStatus::fault?3:2;
            job=planned.value;
            if(!preview_name.empty()) {preview_model.has_value=true; preview_model.value=model.value;}
        }
        const auto serialized = SerializePrintJob(job);
        if(!Usable(serialized)) return serialized.status == ResultStatus::fault ? 3 : 2;
        OptionalValue<std::string> preview;
        if(!preview_name.empty()) {
            const auto html=preview_model.has_value?CreatePreviewHtml(job,preview_model.value):CreatePreviewHtml(job);
            if(!Usable(html)) return html.status==ResultStatus::fault?3:2;
            preview.has_value=true; preview.value=html.value;
        }
        if(output_name.empty()) output_name = Wide(job.job_id + ".json");
        const auto written = WriteNewArtifact(project,output_name,serialized.value);
        Summary(job);
        std::cout<<"Created "<<Json(Utf8(written)).dump()<<std::endl;
        if(preview.has_value) {
            const auto html_written=WriteNewArtifact(project,preview_name,preview.value,true);
            std::cout<<"Created "<<Json(Utf8(html_written)).dump()<<'\n';
        }
        if(stage==L"imported") std::cout<<"robot_model files were not opened; planning settings were not evaluated.\n";
        return 0;
    } catch(const Json::exception&) {
        std::cerr << "Invalid configuration JSON syntax, encoding or types.\n"; return 2;
    } catch(const std::exception& error) {
        std::cerr << Json(error.what()).dump() << '\n'; return 3;
    }
}
