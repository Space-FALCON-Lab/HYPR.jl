# Parse every source file, including functions Julia has not compiled yet.
using TOML, SHA
const JS=Base.JuliaSyntax
function body_bytes(node)
    kids = JS.children(node)
    if JS.kind(node) == JS.K"block" && kids !== nothing && !isempty(kids)
        return first(body_bytes(first(kids))), last(body_bytes(last(kids)))
    end
    text = String(JS.sourcetext(node))
    first_nonspace = findfirst(!isspace, text)
    last_nonspace = findlast(!isspace, text)
    first_nonspace === nothing && return JS.first_byte(node), JS.last_byte(node)
    return JS.first_byte(node) + first_nonspace - 1,
        JS.first_byte(node) + last_nonspace - 1
end

function function_spans!(spans,node)
    if JS.kind(node) in (JS.K"function", JS.K"->", JS.K"do")
        # Julia can attribute a named short-form body to its declaration line.
        # Closures need a narrower span: creation/call can run without the body.
        kids = JS.children(node)
        anonymous = JS.kind(node) != JS.K"function" || JS.kind(first(kids)) == JS.K"tuple"
        body = anonymous ? last(kids) : node
        first_byte, last_byte = anonymous ? body_bytes(body) : (JS.first_byte(body), JS.last_byte(body))
        first_line=JS.source_line(body.source,first_byte)
        last_line=JS.source_line(body.source,last_byte)
        push!(spans,Dict("first"=>first_line,"last"=>last_line,
            "kind"=>string(JS.kind(node)),
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
function main(args=ARGS)
    root=normpath(joinpath(@__DIR__,".."));sources=Dict{String,Any}()
    for dir in ("src","ext"),(path,_,files) in walkdir(joinpath(root,dir)),file in files
        endswith(file,".jl") || continue
        full=joinpath(path,file);text=read(full,String)
        ast=JS.parseall(JS.SyntaxNode,text;filename=full,ignore_errors=false,ignore_warnings=true)
        spans=function_spans!(Dict{String,Any}[],ast)
        sources[relpath(full,root)]=Dict("sha256"=>bytes2hex(sha256(text)),"functions"=>spans,
            "module_wiring_only"=>wiring_only(ast))
    end
    length(args)==1 || error("Provide the output inventory path")
    open(only(args),"w") do io
        TOML.print(io,Dict("sources"=>sources);sorted=true)
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
