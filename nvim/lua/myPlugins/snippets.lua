local ls = require "luasnip"
local s = ls.snippet
local t = ls.text_node
local i = ls.insert_node
local f = ls.function_node

ls.add_snippets("xml", {
    s("xmlns", {
        t "<",
        i(1, "RootElement"),
        t ' xmlns="',
        i(2, "https://namespace.url"),
        t '"',
        t { "", '  xmlns:xsi="http://www.w3.org/2001/XMLSchema"' },
        t { "", '  xsi:schemaLocation="' },
        f(function(args) return args[1][1] end, { 2 }),
        t " ",
        i(3, "https://schema.location.url"),
        t '">',
        t { "", "" },
        t "</",
        f(function(args) return args[1][1] end, { 1 }),
        t ">",
    }),

    -- Pre-defined example for CycloneDDS
    s("cyclonedds", {
        t {
            '<CycloneDDS xmlns="https://cdds.io/config"',
            '  xmlns:xsi="http://www.w3.org/2001/XMLSchema"',
            '  xsi:schemaLocation="https://cdds.io/config ',
        },
        i(1, "https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd"),
        t { '">', "", "</CycloneDDS>" },
    }),
})

-- Add URDF specific snippet
ls.add_snippets("urdf", {
    s("urdfheader", {
        t { '<?xml version="1.0" encoding="UTF-8"?>', '<robot name="' },
        i(1, "my_robot"),
        t {
            '"',
            '  xmlns:xsi="http://www.w3.org/2001/XMLSchema"',
            '  xsi:schemaLocation="http://www.ros.org/urdf/xml ',
        },
        i(2, "https://raw.githubusercontent.com/ros/urdfdom/master/xsd/urdf.xsd"),
        t {
            '">',
            "",
            "  <!-- Links -->",
            "",
            "  <!-- Joints -->",
            "",
            "</robot>",
        },
    }),
})
-- Also add to urdf filetype (which we already mapped to xml)
ls.filetype_extend("urdf", { "xml" })

require("nvim-treesitter.configs").setup {
    -- Your existing config

    -- Add this filetype mapping
    filetype_to_parsername = {
        urdf = "xml",
    },
}
