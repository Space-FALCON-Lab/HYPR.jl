# Parse every source file, including functions Julia has not compiled yet.
using TOML, SHA
const JS=Base.JuliaSyntax
function function_spans!(spans,node)
    if JS.kind(node)==JS.K"function"
        first_line=JS.source_line(node.source,JS.first_byte(node))
        last_line=JS.source_line(node.source,JS.last_byte(node))
        push!(spans,Dict("first"=>first_line,"last"=>last_line,
            "label"=>first(split(String(JS.sourcetext(node)),'\n'))))
    end
    kids=JS.children(node)
    kids===nothing || foreach(n->function_spans!(spans,n),kids)
    spans
end
function wiring_only(node)
    k=JS.kind(node);kids=JS.children(node)
    k in (JS.K"Identifier",JS.K"string",JS.K"using",JS.K"import",JS.K"export") && return true
    k in (JS.K"toplevel",JS.K"module",JS.K"block",JS.K"doc") &&
        return kids===nothing || all(wiring_only,kids)
    k==JS.K"call" && startswith(String(JS.sourcetext(node)),"include(\"") &&
        return length(kids)==2 && JS.kind(kids[1])==JS.K"Identifier" && String(JS.sourcetext(kids[1]))=="include" && JS.kind(kids[2])==JS.K"string"
    return false
end
root=normpath(joinpath(@__DIR__,".."));sources=Dict{String,Any}()
for dir in ("src","ext"),(path,_,files) in walkdir(joinpath(root,dir)),file in files
    endswith(file,".jl") || continue
    full=joinpath(path,file);text=read(full,String)
    ast=JS.parseall(JS.SyntaxNode,text;filename=full,ignore_errors=false,ignore_warnings=true)
    spans=function_spans!(Dict{String,Any}[],ast)
    sources[relpath(full,root)]=Dict("sha256"=>bytes2hex(sha256(text)),"functions"=>spans,
        "module_wiring_only"=>wiring_only(ast))
end
length(ARGS)==1 || error("Provide the output inventory path")
open(only(ARGS),"w") do io
    TOML.print(io,Dict("sources"=>sources);sorted=true)
end
