#pragma once
#include "template.h"
#include "templateElement.h"
#include "templateGroup.h"
#include <vector>
#include <queue>

namespace FabByExample {
class TemplateUtil {
public:
	// Collect any kind of information about all descendants, using the getter given.
	// The getter will be called for every descendant template, and the result concatenated.
	template<typename T>
	static std::vector<T> getForAllDescendants(Template* tmpl, std::function<void(TemplateElement*, std::vector<T>&)> leafGetter, std::function<void(TemplateGroup*, std::vector<T>&)> groupGetter) {
		std::vector<T> result;
		std::queue<Template*> q;
		q.push(tmpl);
		while (!q.empty()) {
			Template* t = q.front();
			q.pop();
			TemplateElement* te = dynamic_cast<TemplateElement*>(t);
			if (te != nullptr) {
				leafGetter(te, result);
			}
			TemplateGroup* tg = dynamic_cast<TemplateGroup*>(t);
			if (tg != nullptr) {
				groupGetter(tg, result);
				for each (Template* tchild in tg->getChildren()) {
					q.push(tchild);
				}
			}
		}
		return result;
	}

	// Collect any kind of information about all descendants, using the getter given.
	// The getter will be called for every descendant template, and the result concatenated.
	static void doForAllDescendants(Template* tmpl, std::function<void(Template*)> func) {
		std::queue<Template*> q;
		q.push(tmpl);
		while (!q.empty()) {
			Template* t = q.front();
			func(t);
			q.pop();
			TemplateGroup* tg = dynamic_cast<TemplateGroup*>(t);
			if (tg != nullptr) {
				for each (Template* tchild in tg->getChildren()) {
					q.push(tchild);
				}
			}
		}
	}

private:
	TemplateUtil(){}
};
}