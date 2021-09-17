const Sequelize = require("sequelize")

module.exports = (sequelize, DataTypes) =>
sequelize.define('schedule', {
    scheduleid: { type: DataTypes.INTEGER, autoIncrement: true, primaryKey: true },
    schedule_time: { type: 'TIMESTAMP', defaultValue: Sequelize.literal('CURRENT_TIMESTAMP')},
    schedule_title: { type: DataTypes.STRING },
    schedule_des: { type: DataTypes.STRING },
});
