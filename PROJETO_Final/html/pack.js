'use strict'

if (process.argv.length !== 3) {
	console.log('Usage: node pack <file>')
	process.exit(1)
}

var file = process.argv[2],
	fs = require('fs'),
	path = require('path'),
	dir = path.dirname(file),
	ext = path.extname(file),
	name = path.basename(file, ext),
	content = fs.readFileSync(file, 'utf8')

content = content.replace(/src="(.+?\.(png|gif))"/g, function (_, src, format) {
	var data = fs.readFileSync(path.join(dir, src))
	return 'src="data:image/' + format + ';base64,' + data.toString('base64') + '"'
})

fs.writeFileSync(path.join(dir, name + '_packed' + ext), content)