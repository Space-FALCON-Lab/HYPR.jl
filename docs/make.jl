using Documenter, HYPR
makedocs(sitename="HYPR.jl",modules=[HYPR],checkdocs=:exports,
    doctest=true,warnonly=false,remotes=nothing,
    format=Documenter.HTML(repolink=nothing, edit_link=nothing, prettyurls=get(ENV,"CI","false")=="true", size_threshold=400_000),
    pages=["Start"=>"index.md","API"=>"api.md","Integration"=>"integration.md",
        "Validation"=>"validation.md","Migration"=>"migration.md"])
