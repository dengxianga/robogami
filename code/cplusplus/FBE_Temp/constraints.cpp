#include "constraints.h"
#include "template.h"

using namespace FabByExample;

bool Constraint::isRelated(Template* _temp) {
	const auto& coeffs = linearExpr.getCoeffs();
	for (auto it = coeffs.begin(); it != coeffs.end(); ++it) {
		if (const_cast<Template*>(static_cast<const Template*>(it->first.owner)) == _temp) {
			return true;
		}
	}
	return false;
}

DebugInfo* Constraint::getDebugInfo() {
	auto info = new DebugInfo();
	info->setTypeName("ConstraintLinearExpr");
	info->setShortDescription(concat() << linearExpr.toString() << " = 0");
	info->putStringProperty("equation", concat() << linearExpr.toString() << " = 0");
	return info;
}