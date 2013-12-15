#include <gflags/gflags.h>
#include <recon.hpp>

#include <iostream>
#include <fstream>
#include <string>

DEFINE_bool(dfa, false, "dump DFA as dot language.");
DEFINE_string(f, "", "obtain pattern from FILE.");
DEFINE_bool(i, false, "ignore case distinctions in both the REGEX and the input files..");
DEFINE_bool(minimizing, true, "minimizing DFA");
DEFINE_bool(complemente, false, "complemented DFA");
DEFINE_string(check, "", "check wheter given text is acceptable or not.");
DEFINE_bool(syntax, false, "print RECON regular expression syntax.");
DEFINE_bool(utf8, false, "use utf8 as internal encoding.");
DEFINE_string(out, "", "output file name.");
DEFINE_bool(size, false, "print the size of the DFA.");
DEFINE_bool(factorial, false, "make langauge as a factorial");

void set_filename(const std::string&, std::string&);

using namespace recon;

int main(int argc, char* argv[])
{
  google::SetUsageMessage(
      "RANS command line tool.\n"
      "Usage: recon REGEX [Flags ...] \n"
      "You can check RECON extended regular expression syntax via '--syntax' option."
                          );
  google::SetVersionString(std::string("build rev: ")+std::string(GIT_REV));
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::string regex;
  if (!FLAGS_f.empty()) {
    std::ifstream ifs(FLAGS_f.data());
    ifs >> regex;
  } else if (argc > 1) {
    regex = argv[1];
  } else {
    std::cout << google::ProgramUsage() << std::endl;
    return -1;
  }
  
  NFA n;
  NFA::State& s0 = n.new_state();
  n.start_states().insert(s0.id);
  NFA::State& s1 = n.new_state();
  NFA::State& s2 = n.new_state();
  NFA::State& s3 = n.new_state();
  s0['a'].insert(s0.id);
  s0['b'].insert(s0.id);
  s0['a'].insert(s1.id);
  s1['a'].insert(s2.id);
  s1['b'].insert(s2.id);
  s2['a'].insert(s3.id);
  s2['b'].insert(s3.id);
  s3.accept = true;
  DFA d(n);

  std::cout << d;
  
  return 0;
}

void set_filename(const std::string& src, std::string& dst)
{
  if (!dst.empty()) return;

  static const std::string suffix = ".rans";

  if (src.length() > suffix.length() &&
      src.substr(src.length() - suffix.length(), std::string::npos) == suffix) {
    dst = src.substr(0, src.length() - suffix.length());
  } else {
    std::string input = "dummy";
    do {
      switch (input[0]) {
        case 'y': case 'A': dst = src; return;
        case 'n': case 'N': return;
        case 'r':
          std::cout << "new name: ";
          std::cin >> dst;
          return;
        default:
          std::cout << "replace " << src << "? [y]es, [n]o, [A]ll, [N]one, [r]ename: ";
      }
    } while (std::cin >> input);
  }
}
