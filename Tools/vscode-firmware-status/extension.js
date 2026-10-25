const vscode = require('vscode');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const ANALYZE = 'stm32cube-ide-build-analyzer.show-memory-analysis-for-one-build-file-identified-by-an-uri';

function activate(context) {
  const output = vscode.window.createOutputChannel('rosDemo 固件状态');
  const item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 110);
  item.command = 'rosdemo.firmware.status';
  context.subscriptions.push(output, item);
  let chain = Promise.resolve();
  let debounce;
  function folder() {
    const candidates=(vscode.workspace.workspaceFolders || []).filter(f =>
      fs.existsSync(path.join(f.uri.fsPath, 'rosDemo.ioc')));
    return candidates.length === 1 ? candidates[0] : undefined;
  }
  async function refresh(showAnalysis = true) {
    const project = folder();
    if (!project) return;
    const root=project.uri.fsPath;
    const file=path.join(root,'.vscode','firmware-status.json');
    if (!fs.existsSync(file)) return;
    const state=JSON.parse(fs.readFileSync(file,'utf8'));
    const build=state.build;
    item.text=`$(circuit-board) 编译 ${build?.version || '未知'} | 烧录 ${state.flash?.version || '未记录'}（用户报告）`;
    item.tooltip=`${build?.preset || ''}\n${build?.elf || ''}\n${state.hardware_note || ''}`;
    item.show();
    if (!build) return;
    const relative=path.relative(path.join(root,'build'),build.elf);
    if (relative.startsWith('..') || path.isAbsolute(relative)) throw new Error('ELF outside project build directory');
    if (crypto.createHash('sha256').update(fs.readFileSync(build.elf)).digest('hex')!==build.sha256)
      throw new Error('ELF changed since the successful build record');
    const presets=JSON.parse(fs.readFileSync(path.join(root,'CMakePresets.json'),'utf8'));
    if (!presets.configurePresets.some(p => p.name===build.preset)) throw new Error('Unknown preset');
    const cmake=vscode.extensions.getExtension('ms-vscode.cmake-tools');
    if (!cmake) throw new Error('CMake Tools unavailable');
    await cmake.activate();
    const current=await vscode.commands.executeCommand('cmake.activeConfigurePresetName');
    if (current!==build.preset) await vscode.commands.executeCommand('cmake.setConfigurePreset',build.preset,project);
    await vscode.commands.executeCommand('cmake.setBuildPreset',build.preset,project);
    const analyzer=vscode.extensions.getExtension('stmicroelectronics.stm32cube-ide-build-analyzer');
    if (!analyzer) throw new Error('STM32 Build Analyzer unavailable');
    await analyzer.activate();
    if (showAnalysis) await vscode.commands.executeCommand(ANALYZE,vscode.Uri.file(build.elf));
    const activePreset=await vscode.commands.executeCommand('cmake.activeConfigurePresetName');
    const receipt={updated_at:new Date().toISOString(),requested_at:state.updated_at,
      active_preset:activePreset,expected_preset:build.preset,analyzer_elf:build.elf,
      analyzer_command_completed:showAnalysis,flash:state.flash,
      success:activePreset===build.preset};
    fs.writeFileSync(path.join(root,'.vscode','firmware-ui-receipt.json'),JSON.stringify(receipt,null,2)+'\n');
    output.appendLine(JSON.stringify(receipt));
    if (!receipt.success) throw new Error('Active CMake preset did not match');
  }
  function queue(show=true) {
    chain=chain.then(()=>refresh(show)).catch(error=>{
      item.text='$(warning) 固件界面尚未同步';
      output.appendLine(String(error.stack || error));
      const project=folder();
      if(project) fs.writeFileSync(path.join(project.uri.fsPath,'.vscode','firmware-ui-error.json'),
        JSON.stringify({at:new Date().toISOString(),error:String(error)},null,2));
    });
    return chain;
  }
  context.subscriptions.push(vscode.commands.registerCommand('rosdemo.firmware.refresh',()=>queue(true)));
  context.subscriptions.push(vscode.commands.registerCommand('rosdemo.firmware.status',async()=>{
    const project=folder();
    if(project) await vscode.commands.executeCommand('vscode.open',vscode.Uri.file(path.join(project.uri.fsPath,'.vscode','firmware-status.json')));
  }));
  context.subscriptions.push(vscode.window.registerUriHandler({handleUri:uri=>
    uri.path==='/reload' ? vscode.commands.executeCommand('workbench.action.reloadWindow') : queue(true)}));
  const project=folder();
  if(project) {
    const watcher=vscode.workspace.createFileSystemWatcher(new vscode.RelativePattern(project,'.vscode/firmware-status.json'));
    const changed=()=>{clearTimeout(debounce);debounce=setTimeout(()=>queue(true),400);};
    context.subscriptions.push(watcher,watcher.onDidChange(changed),watcher.onDidCreate(changed),
      {dispose:()=>clearTimeout(debounce)});
    queue(true);
  }
}
module.exports={activate};
