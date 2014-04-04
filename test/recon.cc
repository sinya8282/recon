#include <gflags/gflags.h>
#include <recon.hpp>

#include <iostream>
#include <fstream>
#include <string>

DEFINE_bool(dfa, false, "dump DFA as a dot language.");
DEFINE_string(f, "", "obtain pattern from FILE.");
DEFINE_string(alphabets, ".", "alphabets of the converted expression.");
DEFINE_bool(i, false, "ignore case distinctions in both the REGEX and the input files..");
DEFINE_bool(minimizing, true, "minimizing DFA");
DEFINE_bool(complemente, false, "complemented DFA");
DEFINE_string(check, "", "check wheter given text is acceptable or not.");
DEFINE_bool(syntax, false, "print RECON regular expression syntax.");
DEFINE_bool(utf8, false, "use utf8 as internal encoding.");
DEFINE_string(out, "", "output file name.");
DEFINE_bool(size, false, "print the size of the DFA.");
DEFINE_bool(expression, false, "");
DEFINE_bool(starfree_expression, false, "");
DEFINE_bool(aperiodic, false, "");
DEFINE_bool(monoid, false, "dump Monoid as a dot language");

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

  RECON r(regex);
  if (!r.alphabets(FLAGS_alphabets)) {
    std::cerr << r.error() << std::endl;;
    return 0;
  }
  
  if (FLAGS_dfa || FLAGS_expression)
  {
    r.scompile();
    if (!r.ok()) {
      std::cerr << "error" << std::endl;
      std::cerr << r.error() << std::endl;
    } else {
      if (FLAGS_dfa) {
        std::cout << r.dfa() << std::endl;
      }
      if (FLAGS_expression) {
        std::cout << r.expression() << std::endl;        
      }
    }
  }

  if (FLAGS_starfree_expression || FLAGS_aperiodic || FLAGS_monoid) {
    r.scompile();
    if (!r.ok()) {
      std::cerr << "error" << std::endl;
      std::cerr << r.error() << std::endl;
    } else {
      if (FLAGS_monoid) {
        std::cout << r.monoid() << std::endl;
      }
      if (FLAGS_aperiodic) {
        std::cout << r.monoid().aperiodic() << std::endl;
      }
      if (FLAGS_starfree_expression) {
      std::cout << r.starfree_expression() << std::endl;
      }
    }
  }
  
  return 0;
}
