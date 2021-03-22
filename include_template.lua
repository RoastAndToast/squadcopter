if not __notFirst__ then
	-- script to include. Path from scene directory. --
    local file='/script.lua'
    local scenePath=simGetStringParameter(sim_stringparam_scene_path)
    __notFirst__=true
    __scriptCodeToRun__=assert(loadfile(scenePath..file))
end
if __scriptCodeToRun__ then
    __scriptCodeToRun__()
end