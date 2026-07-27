#include "../../include/BrakeController.h"
#include "../../include/CableKinematics.h"
#include "../../include/COMPortNode.h"
#include "../../include/GripperController.h"
#include "../../include/SerialPort.h"
#include "../../include/TrajectoryGenerator.h"

#include <cmath>
#include <deque>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

class TestRunner {
private:
    int failureCount{0};

public:
    void Expect(
        bool condition,
        const char* expression,
        const char* file,
        int line
    ) {
        if(condition) return;
        failureCount++;
        std::cerr << file << "(" << line << "): expectation failed: "
                  << expression << std::endl;
    }

    template<typename Actual, typename Expected>
    void ExpectEqual(
        const Actual& actual,
        const Expected& expected,
        const char* actualExpression,
        const char* expectedExpression,
        const char* file,
        int line
    ) {
        if(actual == expected) return;
        failureCount++;
        std::cerr << file << "(" << line << "): expected "
                  << actualExpression << " == " << expectedExpression
                  << ", actual values were [" << actual << "] and ["
                  << expected << "]" << std::endl;
    }

    void ExpectNear(
        double actual,
        double expected,
        double tolerance,
        const char* actualExpression,
        const char* expectedExpression,
        const char* file,
        int line
    ) {
        if(std::abs(actual - expected) <= tolerance) return;
        failureCount++;
        std::cerr << file << "(" << line << "): expected "
                  << actualExpression << " ~= " << expectedExpression
                  << " within " << tolerance << ", actual values were ["
                  << actual << "] and [" << expected << "]" << std::endl;
    }

    int Failures() const {
        return failureCount;
    }

    void Run(const std::string& name, const std::function<void()>& test) {
        const int failuresBefore = failureCount;
        std::ostringstream capturedOutput;
        std::ostringstream capturedErrors;
        std::streambuf* originalOutput = std::cout.rdbuf(
            capturedOutput.rdbuf()
        );
        std::streambuf* originalErrors = std::cerr.rdbuf(
            capturedErrors.rdbuf()
        );
        try{
            test();
        }catch(const std::exception& error){
            failureCount++;
            std::cerr << "Unhandled exception in " << name << ": "
                      << error.what() << std::endl;
        }catch(...){
            failureCount++;
            std::cerr << "Unhandled exception in " << name << std::endl;
        }
        std::cout.rdbuf(originalOutput);
        std::cerr.rdbuf(originalErrors);

        if(failureCount == failuresBefore)
            std::cout << "[PASS] " << name << std::endl;
        else{
            std::cout << capturedOutput.str();
            std::cerr << capturedErrors.str();
            std::cout << "[FAIL] " << name << std::endl;
        }
    }
};

#define EXPECT_TRUE(runner, expression) \
    (runner).Expect((expression), #expression, __FILE__, __LINE__)
#define EXPECT_FALSE(runner, expression) \
    (runner).Expect(!(expression), "!(" #expression ")", __FILE__, __LINE__)
#define EXPECT_EQ(runner, actual, expected) \
    (runner).ExpectEqual( \
        (actual), \
        (expected), \
        #actual, \
        #expected, \
        __FILE__, \
        __LINE__ \
    )
#define EXPECT_NEAR(runner, actual, expected, tolerance) \
    (runner).ExpectNear( \
        (actual), \
        (expected), \
        (tolerance), \
        #actual, \
        #expected, \
        __FILE__, \
        __LINE__ \
    )

class FakeSerialPort final : public ISerialPort {
private:
    static bool NextResult(std::deque<bool>& results) {
        if(results.empty()) return true;
        const bool result = results.front();
        results.pop_front();
        return result;
    }

public:
    bool connected{false};
    bool activeResources{false};
    bool retainResourcesOnConnectFailure{false};
    int connectCalls{0};
    int disconnectCalls{0};
    std::vector<std::string> connectedPorts;
    std::vector<std::string> sentCommands;
    std::deque<bool> connectResults;
    std::deque<bool> disconnectResults;
    std::deque<bool> sendResults;
    std::string readResult;

    bool Connect(const std::string& portName) override {
        connectCalls++;
        connectedPorts.push_back(portName);
        const bool result = NextResult(connectResults);
        connected = result;
        activeResources = result || retainResourcesOnConnectFailure;
        return result;
    }

    bool Disconnect() override {
        disconnectCalls++;
        const bool result = NextResult(disconnectResults);
        if(result){
            connected = false;
            activeResources = false;
        }
        return result;
    }

    bool IsConnected() const override {
        return connected;
    }

    bool HasActiveResources() const override {
        return activeResources;
    }

    std::string Read() override {
        return readResult;
    }

    bool Send(const std::string& command) override {
        sentCommands.push_back(command);
        if(!connected) return false;
        return NextResult(sendResults);
    }
};

std::unique_ptr<ISerialPort> MakeFakeSerial(FakeSerialPort*& fake) {
    std::unique_ptr<FakeSerialPort> serial(new FakeSerialPort());
    fake = serial.get();
    return std::unique_ptr<ISerialPort>(serial.release());
}

void TestOfflineCOMPortLifecycle(TestRunner& runner) {
    COMPortNode port(false);

    EXPECT_FALSE(runner, port.IsConnected());
    EXPECT_FALSE(runner, port.HasActiveResources());
    EXPECT_TRUE(runner, port.Connect("offline"));
    EXPECT_TRUE(runner, port.IsConnected());
    EXPECT_TRUE(runner, port.HasActiveResources());
    EXPECT_FALSE(runner, port.Send("short"));
    EXPECT_TRUE(runner, port.Send("(o,    )"));
    EXPECT_TRUE(runner, port.Disconnect());
    EXPECT_FALSE(runner, port.IsConnected());
    EXPECT_FALSE(runner, port.HasActiveResources());
}

void TestGripperCommandEncoding(TestRunner& runner) {
    FakeSerialPort* fake = nullptr;
    GripperController gripper(
        false,
        true,
        MakeFakeSerial(fake)
    );

    EXPECT_TRUE(runner, gripper.Connect("fake-gripper"));
    EXPECT_TRUE(runner, gripper.IsConnected());
    EXPECT_TRUE(runner, gripper.Open());
    EXPECT_TRUE(runner, gripper.Close());
    EXPECT_TRUE(runner, gripper.Rotate(180));
    EXPECT_TRUE(runner, gripper.Release());

    EXPECT_EQ(runner, fake->connectCalls, 1);
    EXPECT_EQ(runner, fake->sentCommands.size(), static_cast<size_t>(12));
    for(size_t index = 0; index < 3; index++)
        EXPECT_EQ(runner, fake->sentCommands[index], std::string("(o,    )"));
    for(size_t index = 3; index < 6; index++)
        EXPECT_EQ(runner, fake->sentCommands[index], std::string("(c,    )"));
    for(size_t index = 6; index < 9; index++)
        EXPECT_EQ(runner, fake->sentCommands[index], std::string("(d,43  )"));
    for(size_t index = 9; index < 12; index++)
        EXPECT_EQ(runner, fake->sentCommands[index], std::string("(r,    )"));

    EXPECT_TRUE(runner, gripper.Disconnect());
    EXPECT_EQ(runner, fake->disconnectCalls, 1);
    EXPECT_FALSE(runner, gripper.HasActiveResources());
}

void TestGripperFailureAndCleanupRetry(TestRunner& runner) {
    FakeSerialPort* fake = nullptr;
    GripperController gripper(
        false,
        true,
        MakeFakeSerial(fake)
    );
    EXPECT_TRUE(runner, gripper.Connect("fake-gripper"));

    fake->sendResults.push_back(true);
    fake->sendResults.push_back(false);
    fake->sendResults.push_back(true);
    EXPECT_FALSE(runner, gripper.Open());
    EXPECT_EQ(runner, fake->sentCommands.size(), static_cast<size_t>(3));
    EXPECT_FALSE(runner, gripper.IsConnected());
    EXPECT_TRUE(runner, gripper.HasActiveResources());

    fake->disconnectResults.push_back(false);
    fake->disconnectResults.push_back(true);
    EXPECT_FALSE(runner, gripper.Disconnect());
    EXPECT_TRUE(runner, gripper.HasActiveResources());
    EXPECT_TRUE(runner, gripper.Disconnect());
    EXPECT_FALSE(runner, gripper.HasActiveResources());
    EXPECT_EQ(runner, fake->disconnectCalls, 2);
}

void TestFailedConnectRetainsCleanupState(TestRunner& runner) {
    FakeSerialPort* fake = nullptr;
    std::unique_ptr<ISerialPort> serial = MakeFakeSerial(fake);
    fake->connectResults.push_back(false);
    fake->retainResourcesOnConnectFailure = true;
    GripperController gripper(false, true, std::move(serial));

    EXPECT_FALSE(runner, gripper.Connect("fake-gripper"));
    EXPECT_FALSE(runner, gripper.IsConnected());
    EXPECT_TRUE(runner, gripper.HasActiveResources());
    EXPECT_TRUE(runner, gripper.Disconnect());
    EXPECT_FALSE(runner, gripper.HasActiveResources());
}

void TestBrakeBoundsAndPartialOpenRollback(TestRunner& runner) {
    FakeSerialPort* fake = nullptr;
    BrakeController brake(false, MakeFakeSerial(fake));

    EXPECT_FALSE(runner, brake.UseCableBrake(5));
    EXPECT_TRUE(runner, brake.UseCableBrake(3));
    EXPECT_TRUE(runner, brake.Connect("fake-brake"));
    EXPECT_FALSE(runner, brake.OpenCableBrakeByIndex(-1));
    EXPECT_FALSE(runner, brake.OpenCableBrakeByIndex(3));
    EXPECT_TRUE(runner, fake->sentCommands.empty());

    fake->sendResults.push_back(true);
    fake->sendResults.push_back(false);
    fake->sendResults.push_back(true);
    fake->sendResults.push_back(true);
    EXPECT_FALSE(runner, brake.OpenAllCableBrake());

    EXPECT_EQ(runner, fake->sentCommands.size(), static_cast<size_t>(4));
    EXPECT_EQ(runner, fake->sentCommands[0], std::string("(4:0)   "));
    EXPECT_EQ(runner, fake->sentCommands[1], std::string("(5:0)   "));
    EXPECT_EQ(runner, fake->sentCommands[2], std::string("(6:0)   "));
    EXPECT_EQ(runner, fake->sentCommands[3], std::string("(9:1)   "));
    EXPECT_TRUE(runner, brake.Disconnect());
}

void TestLinearTrajectory(TestRunner& runner) {
    double start[6] = {0, 1, 2, 3, 4, 5};
    double end[6] = {1, 2, 3, 4, 5, 6};
    const std::vector<std::vector<double>> trajectory =
        GenLinearTrajForCableMotor(start, end, 100, false);

    EXPECT_EQ(runner, trajectory.size(), static_cast<size_t>(3));
    for(const std::vector<double>& frame : trajectory)
        EXPECT_EQ(runner, frame.size(), static_cast<size_t>(6));
    for(size_t index = 0; index < 6; index++){
        EXPECT_NEAR(runner, trajectory.front()[index], start[index], 1e-12);
        EXPECT_NEAR(runner, trajectory.back()[index], end[index], 1e-12);
        EXPECT_NEAR(
            runner,
            trajectory[1][index],
            (start[index] + end[index]) / 2.0,
            1e-12
        );
    }
}

void TestParabolicTrajectory(TestRunner& runner) {
    double start[6] = {0, 0, 0, 0, 0, 0};
    double end[6] = {0.1, 0.05, -0.02, 0.1, -0.1, 0.05};
    const std::vector<std::vector<double>> trajectory =
        GenParaBlendTrajForCableMotor(start, end, 1000, false);

    EXPECT_EQ(runner, trajectory.size(), static_cast<size_t>(21));
    for(const std::vector<double>& frame : trajectory){
        EXPECT_EQ(runner, frame.size(), static_cast<size_t>(6));
        for(double value : frame)
            EXPECT_TRUE(runner, std::isfinite(value));
    }
    for(size_t index = 0; index < 6; index++){
        EXPECT_NEAR(runner, trajectory.front()[index], start[index], 1e-12);
        EXPECT_NEAR(runner, trajectory.back()[index], end[index], 1e-9);
    }

    EXPECT_TRUE(
        runner,
        GenParaBlendTrajForCableMotor(start, end, 200, false).empty()
    );
    double distantEnd[6] = {10, 0, 0, 0, 0, 0};
    EXPECT_TRUE(
        runner,
        GenParaBlendTrajForCableMotor(start, distantEnd, 250, false).empty()
    );
}

void TestCableKinematics(TestRunner& runner) {
    cu_brick::CableKinematicsConfig config;
    config.cableCount = 2;
    config.pulleyRadius = 0.05;
    config.frameAttachments[0] = Eigen::Vector3d(-1, 0, 0);
    config.frameAttachments[1] = Eigen::Vector3d(1, 0, 0);
    config.endEffectorAttachments[0] = Eigen::Vector3d(-0.1, 0, 0);
    config.endEffectorAttachments[1] = Eigen::Vector3d(0.1, 0, 0);
    config.pulleyAxes[0] = Eigen::Vector3d(0, 0, 1);
    config.pulleyAxes[1] = Eigen::Vector3d(0, 0, 1);

    const std::array<double, 6> nearPose = {0, 0, -2, 0, 0, 0};
    const std::array<double, 6> farPose = {0, 0, -3, 0, 0, 0};
    const std::array<double, cu_brick::kMaximumRailCount> railOffsets =
        {0, 0, 0, 0};
    const std::vector<double> nearLengths =
        cu_brick::ComputeCableLengths(config, nearPose, railOffsets);
    const std::vector<double> farLengths =
        cu_brick::ComputeCableLengths(config, farPose, railOffsets);

    EXPECT_EQ(runner, nearLengths.size(), static_cast<size_t>(2));
    EXPECT_EQ(runner, farLengths.size(), static_cast<size_t>(2));
    EXPECT_NEAR(runner, nearLengths[0], nearLengths[1], 1e-12);
    for(size_t index = 0; index < nearLengths.size(); index++){
        EXPECT_TRUE(runner, std::isfinite(nearLengths[index]));
        EXPECT_TRUE(runner, nearLengths[index] > 0);
        EXPECT_TRUE(runner, farLengths[index] > nearLengths[index]);
    }

    config.cableCount = cu_brick::kMaximumCableCount + 1;
    bool rejectedOversizedConfig = false;
    try{
        cu_brick::ComputeCableLengths(config, nearPose, railOffsets);
    }catch(const std::out_of_range&){
        rejectedOversizedConfig = true;
    }
    EXPECT_TRUE(runner, rejectedOversizedConfig);
}

} // namespace

int main() {
    TestRunner runner;
    runner.Run(
        "offline COM port lifecycle",
        [&runner](){ TestOfflineCOMPortLifecycle(runner); }
    );
    runner.Run(
        "gripper command encoding",
        [&runner](){ TestGripperCommandEncoding(runner); }
    );
    runner.Run(
        "gripper failure and cleanup retry",
        [&runner](){ TestGripperFailureAndCleanupRetry(runner); }
    );
    runner.Run(
        "failed connect retains cleanup state",
        [&runner](){ TestFailedConnectRetainsCleanupState(runner); }
    );
    runner.Run(
        "brake bounds and partial-open rollback",
        [&runner](){ TestBrakeBoundsAndPartialOpenRollback(runner); }
    );
    runner.Run(
        "linear trajectory generation",
        [&runner](){ TestLinearTrajectory(runner); }
    );
    runner.Run(
        "parabolic trajectory generation",
        [&runner](){ TestParabolicTrajectory(runner); }
    );
    runner.Run(
        "cable kinematics",
        [&runner](){ TestCableKinematics(runner); }
    );

    if(runner.Failures() == 0){
        std::cout << "All offline tests passed." << std::endl;
        return 0;
    }
    std::cerr << runner.Failures() << " offline test expectation(s) failed."
              << std::endl;
    return 1;
}
