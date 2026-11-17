#include <array>
#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include <systemc>
#include <uvm>
#include <boost/property_tree/xml_parser.hpp>
#include <fc4sc.hpp>
#include <crave/ConstrainedRandom.hpp>
#include <crave/experimental/Experimental.hpp>
#include <crave/utils/Logging.hpp>

#include "jbfifo.h"

using namespace sc_core;
using namespace uvm;

namespace {

constexpr int kWidth = 16;
constexpr int kDepth = 8;
// Count the memory, din and rdat, and the two named TLM channel slots.
constexpr unsigned kCapacity = kDepth + 4;
const sc_time kClockPeriod(1, SC_NS);
const sc_time kQuietWindow(16, SC_NS);
using Data = ac_int<kWidth, false>;

struct FifoPayload : crave::crv_sequence_item {
    static constexpr unsigned value_count = 1u << kWidth;
    crave::crv_variable<unsigned> value{"value"};
    crave::crv_constraint width{
        "width", crave::dist(value(), crave::make_distribution(crave::range<unsigned>(0, value_count - 1)))
    };
    crave::crv_constraint unique_values{"unique_values"};

    explicit FifoPayload(crave::crv_object_name) {}

    bool next(unsigned& word) {
        if (!randomize())
            return false;
        word = value;
        unique_values &= crave::crv_constraint(value() != word);
        return true;
    }
};

enum class FifoOperation { push, pop, probe_full, expect_empty };
enum class FifoEventKind { input, output, input_blocked, drain_complete, status };

struct FifoEvent {
    FifoEventKind kind;
    Data data = 0;
    bool empty = false;
    bool full = false;
};

struct FifoInterface : sc_module {
    sc_clock clk{"clk", kClockPeriod};
    sc_signal<bool> rstn{"rstn"};
    sc_signal<bool> empty{"empty"};
    sc_signal<bool> full{"full"};
    Connections::Combinational<Data> input{"input"};
    Connections::Combinational<Data> output{"output"};
    tlm::tlm_analysis_port<FifoEvent> observations{"observations"};

    SC_HAS_PROCESS(FifoInterface);
    explicit FifoInterface(sc_module_name name) : sc_module(name) {
        SC_THREAD(reset);
        sensitive << clk.posedge_event();
    }

    void push(Data data) {
        input.Push(data);
        observe_input(data);
    }

    Data pop() {
        const Data data = output.Pop();
        output_data.write(data.to_uint());
        output_count.write(output_count.read() + 1);
        observations.write({FifoEventKind::output, data});
        return data;
    }

    bool probe_full(Data data) {
        const sc_time deadline = sc_time_stamp() + kQuietWindow;
        probe_waiting.write(true);
        while (!input.PushNB(data)) {
            if (sc_time_stamp() >= deadline) {
                probe_waiting.write(false);
                observations.write({FifoEventKind::input_blocked, data, empty.read(), full.read()});
                return false;
            }
            wait(deadline - sc_time_stamp(), input.fifo.ok_to_put());
        }
        probe_waiting.write(false);
        observe_input(data);
        return true;
    }

    bool output_empty() {
        wait(kQuietWindow);
        observations.write({FifoEventKind::drain_complete, 0, empty.read(), full.read()});
        Data data;
        return !output.PeekNB(data);
    }

    void trace(sc_trace_file* file) const {
        sc_trace(file, clk, "fifo.clk");
        sc_trace(file, rstn, "fifo.rstn");
        sc_trace(file, empty, "fifo.o_empty");
        sc_trace(file, full, "fifo.o_full");
        sc_trace(file, input_data, "fifo.tlm_input_data");
        sc_trace(file, output_data, "fifo.tlm_output_data");
        sc_trace(file, input_count, "fifo.tlm_input_count");
        sc_trace(file, output_count, "fifo.tlm_output_count");
        sc_trace(file, probe_waiting, "fifo.saturation_probe_waiting");
    }

private:
    // Each counter change marks a completed TLM transfer, including repeated data.
    sc_signal<sc_dt::sc_uint<kWidth>> input_data{"tlm_input_data"};
    sc_signal<sc_dt::sc_uint<kWidth>> output_data{"tlm_output_data"};
    sc_signal<unsigned> input_count{"tlm_input_count"};
    sc_signal<unsigned> output_count{"tlm_output_count"};
    sc_signal<bool> probe_waiting{"saturation_probe_waiting"};

    void observe_input(Data data) {
        input_data.write(data.to_uint());
        input_count.write(input_count.read() + 1);
        observations.write({FifoEventKind::input, data});
    }

    void reset() {
        rstn.write(false);
        input.ResetWrite();
        output.ResetRead();
        wait(3);
        wait(clk.negedge_event());
        rstn.write(true);
    }
};

struct FifoTbTop : sc_module {
    FifoInterface vif{"vif"};

    explicit FifoTbTop(sc_module_name name) : sc_module(name) {
        dut.clk(vif.clk);
        dut.rstn(vif.rstn);
        dut.ch_din(vif.input);
        dut.ch_dout(vif.output);
        dut.o_empty(vif.empty);
        dut.o_full(vif.full);
    }

private:
    JbFifo<kWidth, kDepth> dut{"dut"};
};

struct FifoRequest : uvm_sequence_item {
    using uvm_sequence_item::uvm_sequence_item;
    UVM_OBJECT_UTILS(FifoRequest);

    FifoOperation operation = FifoOperation::push;
    Data data = 0;
};

struct FifoResponse : uvm_sequence_item {
    using uvm_sequence_item::uvm_sequence_item;
    UVM_OBJECT_UTILS(FifoResponse);

    bool success = false;
};

using InterfaceConfig = uvm_config_db<FifoInterface*>;

class FifoSequencer : public uvm_sequencer<FifoRequest, FifoResponse> {
public:
    explicit FifoSequencer(uvm_component_name name) : uvm_sequencer(name) {}
    UVM_COMPONENT_UTILS(FifoSequencer);
};

class FifoDriver : public uvm_driver<FifoRequest, FifoResponse> {
public:
    explicit FifoDriver(uvm_component_name name) : uvm_driver(name) {}
    UVM_COMPONENT_UTILS(FifoDriver);

    void build_phase(uvm_phase&) override {
        if (!InterfaceConfig::get(this, "", "vif", vif)) {
            UVM_FATAL("CONFIG", "Missing FIFO interface for driver");
        }
    }

    void run_phase(uvm_phase&) override {
        if (!vif->rstn.read())
            wait(vif->rstn.posedge_event());

        while (true) {
            FifoRequest request;
            seq_item_port->get_next_item(request);
            bool success = true;
            switch (request.operation) {
            case FifoOperation::push:
                vif->push(request.data);
                break;
            case FifoOperation::pop:
                vif->pop();
                break;
            case FifoOperation::probe_full:
                success = vif->probe_full(request.data);
                break;
            case FifoOperation::expect_empty:
                success = vif->output_empty();
                if (!success) {
                    UVM_ERROR("UNEXPECTED_OUTPUT", "Data remained on the output after draining the FIFO");
                }
                break;
            }

            auto* response = FifoResponse::type_id::create("response", this);
            response->set_id_info(request);
            response->success = success;
            seq_item_port->item_done(*response);
            FifoResponse::type_id::destroy(response);
        }
    }

private:
    FifoInterface* vif = nullptr;
};

class FifoMonitor : public uvm_monitor, public tlm::tlm_analysis_if<FifoEvent> {
public:
    uvm_analysis_export<FifoEvent> observations{"observations"};
    uvm_analysis_port<FifoEvent> transfers{"transfers"};

    explicit FifoMonitor(uvm_component_name name) : uvm_monitor(name) {
        observations.bind(*this);
    }
    UVM_COMPONENT_UTILS(FifoMonitor);

    void build_phase(uvm_phase&) override {
        if (!InterfaceConfig::get(this, "", "vif", vif)) {
            UVM_FATAL("CONFIG", "Missing FIFO interface for monitor");
        }
    }

    void connect_phase(uvm_phase&) override {
        vif->observations.bind(observations);
    }

    void run_phase(uvm_phase&) override {
        if (!vif->rstn.read())
            wait(vif->rstn.posedge_event());
        // The DUT first writes its status from the clocked loop after reset.
        wait(vif->clk.posedge_event());
        wait(SC_ZERO_TIME);
        while (true) {
            transfers.write({FifoEventKind::status, 0, vif->empty.read(), vif->full.read()});
            wait(vif->empty.value_changed_event() | vif->full.value_changed_event());
        }
    }

    void write(const FifoEvent& event) override {
        transfers.write(event);
    }

private:
    FifoInterface* vif = nullptr;
};

enum class CoverageBin {
    input, output, empty, partial, full, input_blocked, input_resumed,
    status_empty, status_partial, status_full
};

class FifoCovergroup : public fc4sc::covergroup {
public:
    CG_CONS(FifoCovergroup) {}

    static constexpr std::array names{
        "input", "output", "empty", "partial", "full", "input_blocked",
        "input_resumed", "status_empty", "status_partial", "status_full"
    };

    COVERPOINT(int, fifo_events, sample_value) {
        fc4sc::bin<int>(names[0], static_cast<int>(CoverageBin::input)),
        fc4sc::bin<int>(names[1], static_cast<int>(CoverageBin::output)),
        fc4sc::bin<int>(names[2], static_cast<int>(CoverageBin::empty)),
        fc4sc::bin<int>(names[3], static_cast<int>(CoverageBin::partial)),
        fc4sc::bin<int>(names[4], static_cast<int>(CoverageBin::full)),
        fc4sc::bin<int>(names[5], static_cast<int>(CoverageBin::input_blocked)),
        fc4sc::bin<int>(names[6], static_cast<int>(CoverageBin::input_resumed)),
        fc4sc::bin<int>(names[7], static_cast<int>(CoverageBin::status_empty)),
        fc4sc::bin<int>(names[8], static_cast<int>(CoverageBin::status_partial)),
        fc4sc::bin<int>(names[9], static_cast<int>(CoverageBin::status_full))
    };

    void record(CoverageBin bin) {
        sample_value = static_cast<int>(bin);
        sample();
    }

private:
    int sample_value = 0;
};

class FifoCoverage : public uvm_subscriber<FifoEvent> {
public:
    explicit FifoCoverage(uvm_component_name name) : uvm_subscriber(name) {}
    UVM_COMPONENT_UTILS(FifoCoverage);

    void write(const FifoEvent& event) override {
        switch (event.kind) {
        case FifoEventKind::input:
            ++outstanding;
            coverage.record(CoverageBin::input);
            if (waiting_for_input && drained_after_block && event.data == pending) {
                coverage.record(CoverageBin::input_resumed);
                waiting_for_input = false;
            }
            break;
        case FifoEventKind::output:
            --outstanding;
            coverage.record(CoverageBin::output);
            drained_after_block = true;
            break;
        case FifoEventKind::input_blocked:
            coverage.record(CoverageBin::input_blocked);
            pending = event.data;
            waiting_for_input = true;
            drained_after_block = false;
            break;
        case FifoEventKind::status:
            if (event.empty && !event.full)
                coverage.record(CoverageBin::status_empty);
            else if (event.full && !event.empty)
                coverage.record(CoverageBin::status_full);
            else if (!event.empty && !event.full)
                coverage.record(CoverageBin::status_partial);
            return;
        case FifoEventKind::drain_complete:
            return;
        }
        if (outstanding == 0)
            coverage.record(CoverageBin::empty);
        else if (outstanding > 0 && outstanding < static_cast<int>(kCapacity))
            coverage.record(CoverageBin::partial);
        else if (outstanding == static_cast<int>(kCapacity))
            coverage.record(CoverageBin::full);
    }

    void report_phase(uvm_phase&) override {
        unsigned covered = 0;
        std::string counts;
        for (unsigned i = 0; i < FifoCovergroup::names.size(); ++i) {
            const auto hits = coverage.fifo_events.get_bin_hit_count(i);
            covered += hits != 0;
            counts += " " + std::string(FifoCovergroup::names[i]) + "=" + std::to_string(hits);
        }
        UVM_INFO("FIFO_COVERAGE", std::to_string(covered) + "/" +
                 std::to_string(FifoCovergroup::names.size()) + " bins hit." + counts, UVM_LOW);

        std::string path;
        if (uvm_config_db<std::string>::get(this, "", "ucis_path", path)) {
            save_ucis(path, get_parent()->get_parent()->get_parent()->get_name());
        }
    }

private:
    void save_ucis(const std::filesystem::path& path, const std::string& test_name) {
        namespace pt = boost::property_tree;
        auto temporary = path;
        temporary += ".tmp";
        try {
            {
                std::ofstream file;
                file.exceptions(std::ios::failbit | std::ios::badbit);
                file.open(temporary);
                xml_printer::coverage_save(file);
                file.close();
            }

            pt::ptree document;
            pt::read_xml(temporary.string(), document, pt::xml_parser::trim_whitespace);
            // FC4SC emits a placeholder test history whose parent is itself.
            auto& history = document.get_child("UCIS.historyNodes.<xmlattr>");
            if (history.get_optional<std::string>("parentId") ==
                history.get_optional<std::string>("historyNodeId")) {
                history.erase("parentId");
            }
            history.put("logicalName", test_name);
            history.put("physicalName", path.filename().string());
            pt::write_xml(temporary.string(), document, std::locale(),
                          pt::xml_writer_make_settings<std::string>(' ', 2));
            std::filesystem::rename(temporary, path);
        } catch (const std::exception& error) {
            std::error_code ignored;
            std::filesystem::remove(temporary, ignored);
            UVM_ERROR("FIFO_UCIS", "Could not write " + path.string() + ": " + error.what());
            return;
        }
        UVM_INFO("FIFO_UCIS", "Wrote " + path.string(), UVM_LOW);
    }

    FifoCovergroup coverage{"fifo"};
    int outstanding = 0;
    bool waiting_for_input = false;
    bool drained_after_block = false;
    Data pending = 0;
};

class FifoScoreboard : public uvm_scoreboard, public tlm::tlm_analysis_if<FifoEvent> {
public:
    uvm_analysis_export<FifoEvent> transfers{"transfers"};
    unsigned expected_count = 0;
    bool expect_saturation = false;

    explicit FifoScoreboard(uvm_component_name name) : uvm_scoreboard(name) {
        transfers.bind(*this);
    }
    UVM_COMPONENT_UTILS(FifoScoreboard);

    void build_phase(uvm_phase&) override {
        coverage = FifoCoverage::type_id::create("coverage", this);
    }

    void connect_phase(uvm_phase&) override {
        coverage_events.connect(coverage->analysis_export);
    }

    void write(const FifoEvent& event) override {
        coverage_events.write(event);
        if (event.kind == FifoEventKind::status) {
            if (event.empty && event.full) {
                UVM_ERROR("FIFO_STATUS", "o_empty and o_full are both asserted");
            }
            return;
        }
        if (event.kind == FifoEventKind::drain_complete) {
            if (!expected.empty() || !event.empty || event.full) {
                UVM_ERROR("FIFO_STATUS", "After drain, expected no pending words, o_empty=1 and o_full=0");
            }
            return;
        }
        if (event.kind == FifoEventKind::input_blocked) {
            if (expected.size() != kCapacity) {
                UVM_ERROR("FIFO_FULL", "Input blocked with " + std::to_string(expected.size()) +
                          " outstanding words, expected " + std::to_string(kCapacity));
            }
            if (event.empty || !event.full) {
                UVM_ERROR("FIFO_STATUS", "At saturation, expected o_empty=0 and o_full=1");
            }
            ++blocked;
            pending = event.data;
            outputs_at_block = outputs;
            return;
        }
        if (event.kind == FifoEventKind::input) {
            if (blocked != resumed) {
                if (outputs == outputs_at_block || event.data != pending) {
                    UVM_ERROR("FIFO_RESUME", "Expected the blocked word to be accepted after an output");
                } else {
                    ++resumed;
                }
            }
            expected.push_back(event.data);
            ++inputs;
            if (expected.size() > kCapacity) {
                UVM_ERROR("FIFO_OVERFLOW", "Accepted more than " + std::to_string(kCapacity) +
                          " outstanding words");
            }
            return;
        }

        ++outputs;
        if (expected.empty()) {
            UVM_ERROR("UNEXPECTED_OUTPUT", "Received data with no pending input");
            return;
        }
        if (event.data != expected.front()) {
            UVM_ERROR("FIFO_ORDER", "Output #" + std::to_string(outputs) +
                      " expected " + std::to_string(expected.front().to_uint()) +
                      ", got " + std::to_string(event.data.to_uint()));
        }
        expected.pop_front();
    }

    void check_phase(uvm_phase&) override {
        const std::string counts = "Expected " + std::to_string(expected_count) +
            " transfers each way, observed " + std::to_string(inputs) +
            " inputs and " + std::to_string(outputs) + " outputs";
        if (inputs != expected_count || outputs != expected_count || !expected.empty()) {
            UVM_ERROR("FIFO_COUNT", counts + ", pending " + std::to_string(expected.size()));
        } else {
            UVM_INFO("FIFO_COUNT", counts, UVM_LOW);
        }
        if (expect_saturation) {
            if (blocked != 1 || resumed != 1) {
                UVM_ERROR("FIFO_FULL", "Expected one saturation and recovery, observed " +
                          std::to_string(blocked) + " blocked and " + std::to_string(resumed) + " resumed");
            } else {
                UVM_INFO("FIFO_FULL", "Input blocked at " + std::to_string(kCapacity) +
                         " outstanding words for " + kQuietWindow.to_string() +
                         " and was accepted after an output", UVM_LOW);
            }
        }
    }

private:
    FifoCoverage* coverage = nullptr;
    uvm_analysis_port<FifoEvent> coverage_events{"coverage_events"};

    std::deque<Data> expected;
    unsigned inputs = 0;
    unsigned outputs = 0;
    unsigned blocked = 0;
    unsigned resumed = 0;
    unsigned outputs_at_block = 0;
    Data pending = 0;
};

class FifoVirtualSequencer : public uvm_sequencer<> {
public:
    explicit FifoVirtualSequencer(uvm_component_name name) : uvm_sequencer(name) {}
    UVM_COMPONENT_UTILS(FifoVirtualSequencer);

    FifoSequencer* input = nullptr;
    FifoSequencer* output = nullptr;
};

class FifoEnv : public uvm_env {
public:
    FifoVirtualSequencer* virtual_sequencer = nullptr;
    FifoScoreboard* scoreboard = nullptr;

    explicit FifoEnv(uvm_component_name name) : uvm_env(name) {}
    UVM_COMPONENT_UTILS(FifoEnv);

    void build_phase(uvm_phase&) override {
        input_sequencer = FifoSequencer::type_id::create("input_sequencer", this);
        output_sequencer = FifoSequencer::type_id::create("output_sequencer", this);
        input_driver = FifoDriver::type_id::create("input_driver", this);
        output_driver = FifoDriver::type_id::create("output_driver", this);
        virtual_sequencer = FifoVirtualSequencer::type_id::create("virtual_sequencer", this);
        monitor = FifoMonitor::type_id::create("monitor", this);
        scoreboard = FifoScoreboard::type_id::create("scoreboard", this);
    }

    void connect_phase(uvm_phase&) override {
        virtual_sequencer->input = input_sequencer;
        virtual_sequencer->output = output_sequencer;
        input_driver->seq_item_port.connect(input_sequencer->seq_item_export);
        output_driver->seq_item_port.connect(output_sequencer->seq_item_export);
        monitor->transfers.connect(scoreboard->transfers);
    }

private:
    FifoSequencer* input_sequencer = nullptr;
    FifoSequencer* output_sequencer = nullptr;
    FifoDriver* input_driver = nullptr;
    FifoDriver* output_driver = nullptr;
    FifoMonitor* monitor = nullptr;
};

class FifoSequence : public uvm_sequence<FifoRequest, FifoResponse> {
public:
    explicit FifoSequence(const std::string& name) : uvm_sequence(name) {}
    UVM_OBJECT_UTILS(FifoSequence);

    FifoOperation operation = FifoOperation::push;
    std::vector<Data> words;
    unsigned count = 1;
    bool success = false;

    void body() override {
        for (unsigned i = 0; i < count; ++i) {
            auto* request = FifoRequest::type_id::create("request", get_sequencer());
            request->operation = operation;
            if (!words.empty())
                request->data = words[i];
            start_item(request);
            finish_item(request);
            FifoResponse response;
            get_response(&response);
            FifoRequest::type_id::destroy(request);
            success = response.success;
        }
    }
};

enum class FifoScenario { order, full };

class FifoVirtualSequence : public uvm_sequence<> {
public:
    explicit FifoVirtualSequence(const std::string& name) : uvm_sequence(name) {}
    UVM_OBJECT_UTILS(FifoVirtualSequence);

    FifoScenario scenario = FifoScenario::order;

    void body() override;

    void pre_body() override {
        sequencer = dynamic_cast<FifoVirtualSequencer*>(get_sequencer());
        if (!sequencer || !sequencer->input || !sequencer->output) {
            UVM_FATAL("CONFIG", "FIFO virtual sequence requires both port sequencers");
        }
        std::uint32_t seed = 1;
        uvm_config_db<std::uint32_t>::get(sequencer, "", "seed", seed);
        // CRAVE reserves zero for a time-based seed. Keep FIFO_SEED=0 repeatable.
        const auto crave_seed = seed == 0 ? 1 : seed;
        crave::set_global_seed(crave_seed);
        payload = std::make_unique<FifoPayload>("payload");
        generated_words = 0;
        UVM_INFO("FIFO_SEED", "Payload seed " + std::to_string(seed) +
                 ", CRAVE seed " + std::to_string(crave_seed), UVM_LOW);
    }

private:
    friend class OrderSequence;
    friend class FullSequence;

    template <typename Sequence>
    void run_scenario(const char* name) {
        auto* sequence = Sequence::type_id::create(name, sequencer);
        sequence->start(sequencer, this);
        Sequence::type_id::destroy(sequence);
    }

    std::vector<Data> random_words(unsigned count) {
        if (count > FifoPayload::value_count - generated_words) {
            UVM_FATAL("FIFO_RANDOM", "Requested more unique words than the data width allows");
        }

        std::vector<Data> words;
        words.reserve(count);
        for (unsigned i = 0; i < count; ++i) {
            unsigned value = 0;
            if (!payload->next(value)) {
                UVM_FATAL("FIFO_RANDOM", "CRAVE could not solve the FIFO payload constraints");
            }
            words.emplace_back(value);
            ++generated_words;
        }
        return words;
    }

    bool send(const std::vector<Data>& words, FifoOperation operation = FifoOperation::push) {
        return run_transfer(sequencer->input, "send", operation, words.size(), words);
    }

    void drain(unsigned count) {
        run_transfer(sequencer->output, "drain", FifoOperation::pop, count);
    }

    void check_empty() {
        run_transfer(sequencer->output, "check_empty", FifoOperation::expect_empty, 1);
    }

    void stream(const std::vector<Data>& words, unsigned read_count) {
        sc_join join;
        join.add_process(sc_spawn([this, &words] { send(words); }));
        drain(read_count);
        if (join.process_count() != 0)
            join.wait();
    }

    bool run_transfer(FifoSequencer* port, const char* name, FifoOperation operation,
                      unsigned count, const std::vector<Data>& words = {}) {
        auto* sequence = FifoSequence::type_id::create(name, port);
        sequence->operation = operation;
        sequence->count = count;
        sequence->words = words;
        sequence->start(port, this);
        const bool success = sequence->success;
        FifoSequence::type_id::destroy(sequence);
        return success;
    }

    FifoVirtualSequencer* sequencer = nullptr;
    std::unique_ptr<FifoPayload> payload;
    unsigned generated_words = 0;
};

class OrderSequence : public uvm_sequence<> {
public:
    explicit OrderSequence(const std::string& name) : uvm_sequence(name) {}
    UVM_OBJECT_UTILS(OrderSequence);

    static constexpr unsigned stream_count = 8 * kDepth;
    static constexpr unsigned expected_count = 3 * kDepth + stream_count;

    void body() override {
        auto* parent = dynamic_cast<FifoVirtualSequence*>(get_parent_sequence());
        if (!parent) {
            UVM_FATAL("CONFIG", "OrderSequence requires a FifoVirtualSequence parent");
        }
        parent->send(parent->random_words(kDepth));
        parent->drain(kDepth);

        parent->send(parent->random_words(kDepth));
        parent->drain(kDepth);

        parent->send(parent->random_words(3));
        parent->drain(2);
        parent->send(parent->random_words(kDepth - 3));
        parent->drain(kDepth - 2);

        const auto words = parent->random_words(stream_count);
        const sc_time start = sc_time_stamp();
        parent->stream(words, stream_count);
        const double words_per_clock = stream_count * (kClockPeriod / (sc_time_stamp() - start));
        UVM_INFO("FIFO_THROUGHPUT", std::to_string(stream_count) + " words in " +
                 (sc_time_stamp() - start).to_string() + ", " +
                 std::to_string(words_per_clock) + " words/clock", UVM_LOW);
        parent->check_empty();
    }
};

class FullSequence : public uvm_sequence<> {
public:
    explicit FullSequence(const std::string& name) : uvm_sequence(name) {}
    UVM_OBJECT_UTILS(FullSequence);

    static constexpr unsigned expected_count = kCapacity + 1;

    void body() override {
        auto* parent = dynamic_cast<FifoVirtualSequence*>(get_parent_sequence());
        if (!parent) {
            UVM_FATAL("CONFIG", "FullSequence requires a FifoVirtualSequence parent");
        }
        parent->send(parent->random_words(kCapacity));
        const auto pending = parent->random_words(1);
        if (parent->send(pending, FifoOperation::probe_full)) {
            parent->drain(expected_count);
        } else {
            parent->stream(pending, expected_count);
        }
        parent->check_empty();
    }
};

void FifoVirtualSequence::body() {
    switch (scenario) {
    case FifoScenario::order:
        run_scenario<OrderSequence>("order");
        break;
    case FifoScenario::full:
        run_scenario<FullSequence>("full");
        break;
    }
}

template <FifoScenario Scenario>
class FifoTest : public uvm_test {
public:
    explicit FifoTest(uvm_component_name name) : uvm_test(name) {}

    void build_phase(uvm_phase&) override {
        env = FifoEnv::type_id::create("env", this);
    }

    void run_phase(uvm_phase& phase) override {
        phase.raise_objection(this);
        constexpr bool expect_saturation = Scenario == FifoScenario::full;
        env->scoreboard->expected_count = expect_saturation ?
            FullSequence::expected_count : OrderSequence::expected_count;
        env->scoreboard->expect_saturation = expect_saturation;
        auto* sequence = FifoVirtualSequence::type_id::create("sequence", env->virtual_sequencer);
        sequence->scenario = Scenario;
        sequence->start(env->virtual_sequencer);
        FifoVirtualSequence::type_id::destroy(sequence);
        phase.drop_objection(this);
    }

private:
    FifoEnv* env = nullptr;
};

class FifoOrderTest : public FifoTest<FifoScenario::order> {
public:
    explicit FifoOrderTest(uvm_component_name name) : FifoTest(name) {}
    UVM_COMPONENT_UTILS(FifoOrderTest);
};

class FifoFullTest : public FifoTest<FifoScenario::full> {
public:
    explicit FifoFullTest(uvm_component_name name) : FifoTest(name) {}
    UVM_COMPONENT_UTILS(FifoFullTest);
};

class FifoRandomizeTest : public uvm_test {
public:
    explicit FifoRandomizeTest(uvm_component_name name) : uvm_test(name) {}
    UVM_COMPONENT_UTILS(FifoRandomizeTest);

    void run_phase(uvm_phase& phase) override {
        phase.raise_objection(this);
        crave::set_global_seed(42);
        FifoPayload payload("payload");
        std::unordered_set<unsigned> seen;
        for (unsigned i = 0; i < OrderSequence::expected_count; ++i) {
            unsigned word = 0;
            if (!payload.next(word)) {
                UVM_FATAL("FIFO_RANDOM", "CRAVE failed to generate a payload");
            }
            if (word >= FifoPayload::value_count || !seen.insert(word).second) {
                UVM_ERROR("FIFO_RANDOM", "Out-of-range or duplicate payload " + std::to_string(word));
            }
        }
        if (payload.randomize_with(payload.value() == 0u, payload.value() == 1u)) {
            UVM_ERROR("FIFO_RANDOM", "CRAVE accepted contradictory constraints");
        }
        phase.drop_objection(this);
    }
};

} // namespace

int sc_main(int argc, char** argv) {
    // Keep CRAVE warnings and errors while hiding solver progress messages.
#ifdef CRAVE_HAVE_GLOG
    FLAGS_minloglevel = 1;  // WARNING
#else
    crave::logging_severity = crave::WARNING;
#endif

    if (const char* seed_text = std::getenv("FIFO_SEED")) {
        const std::string_view value(seed_text);
        std::uint32_t seed = 0;
        const auto end = value.data() + value.size();
        const auto result = std::from_chars(value.data(), end, seed);
        if (result.ec != std::errc{} || result.ptr != end) {
            std::cerr << "FIFO_SEED must be an unsigned 32-bit decimal integer\n";
            return EXIT_FAILURE;
        }
        uvm_config_db<std::uint32_t>::set(nullptr, "*.env.virtual_sequencer", "seed", seed);
    }

    FifoTbTop top("fifo");
    std::unique_ptr<sc_trace_file, decltype(&sc_close_vcd_trace_file)>
        waveform(nullptr, sc_close_vcd_trace_file);
    if (argc > 2) {
        waveform.reset(sc_create_vcd_trace_file(argv[2]));
        if (!waveform)
            return EXIT_FAILURE;
        waveform->set_time_unit(1, SC_PS);
        top.vif.trace(waveform.get());
    }
    if (argc > 3)
        uvm_config_db<std::string>::set(nullptr, "*.env.scoreboard.coverage", "ucis_path", argv[3]);
    InterfaceConfig::set(nullptr, "*.env.*", "vif", &top.vif);
    uvm_root::get()->set_timeout(sc_time(100, SC_US));
    run_test(argc > 1 ? argv[1] : "FifoOrderTest");
    const auto* reports = uvm_report_server::get_server();
    return reports->get_severity_count(UVM_ERROR) != 0 ||
           reports->get_severity_count(UVM_FATAL) != 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}
