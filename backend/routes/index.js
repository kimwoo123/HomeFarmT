var express = require('express');
var router = express.Router();
var path = require('path');
var fs = require('fs');


/* GET home page. */
router.get('/map1', function(req, res) {
  const result = {
    conte: 'hello'
  }
  const obj = fs.readFileSync('./public/maps/map1.txt')
  fs.readFile("./public/maps/map1.txt", "utf8", function(err, html) {
    res.send(html);
  })
});

module.exports = router;
