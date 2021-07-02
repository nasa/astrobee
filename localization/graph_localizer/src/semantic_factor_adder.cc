#include <graph_localizer/semantic_factor_adder.h>

namespace graph_localizer {
namespace go = graph_optimizer;
SemanticFactorAdder::SemanticFactorAdder(const SemanticFactorAdderParams& params,
                                         std::shared_ptr<const SemanticObjectTracker> object_tracker)
                                        : SemanticFactorAdder::Base(params), object_tracker_(object_tracker) {
}

std::vector<go::FactorsToAdd> SemanticFactorAdder::AddFactors() {
  go::FactorsToAdd factors_to_add(go::GraphActionCompleterType::SmartFactor);
  return {factors_to_add};
}
} // namespace graph_localizer
