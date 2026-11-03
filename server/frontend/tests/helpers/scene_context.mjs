// Load the real scene-family selection functions into isolated app-handler tests.
import vm from "node:vm";
import { groupSceneTemplates, libraryFamilyId, resolveSceneTask } from "../../library_templates.js";

export function installSceneSelection(context, source) {
  Object.assign(context, { groupSceneTemplates, libraryFamilyId, resolveSceneTask });
  context.visibleSceneTemplates ??= [];
  context.initialJointCatalog ??= null;
  vm.runInContext(source.slice(source.indexOf("function selectedSceneTemplate()"),
    source.indexOf("function configureInitialJoints()")), context);
}
