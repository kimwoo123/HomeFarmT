const Sequelize = require("sequelize");

module.exports = (sequelize, DataTypes) =>
sequelize.define('history', {
    historyid: { type: DataTypes.INTEGER, autoIncrement: true, primaryKey: true },
    event_time: { type: DataTypes.STRING },
    event_title: { type: DataTypes.STRING },
    event_desc: { type: DataTypes.STRING },
},{
    timestamps: false
});